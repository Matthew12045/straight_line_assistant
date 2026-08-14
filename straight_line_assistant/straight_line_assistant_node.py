#!/usr/bin/env python3
#
# Straight Line Assistant + TurtleBot3 Teleop (single node)
#
# Combines the standard ROBOTIS turtlebot3_teleop_keyboard controls
# (w/a/s/d/x with incremental velocity steps) with a PID heading-hold
# controller. A background thread reads keys; a fixed-rate timer
# (default 20 Hz) runs the PID loop and publishes to /cmd_vel so the
# robot always receives a steady command stream — no more stuttering
# from bursty keyboard input.

import os
import math
import sys
import select
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

if os.name != 'nt':
    import termios
    import tty


# ── TurtleBot3 hardware limits ──────────────────────────────────────
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


def euler_from_quaternion(q):
    """Convert quaternion to euler yaw (Z-axis rotation)."""
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)


def make_simple_profile(output, input_val, slop):
    """Smoothly ramp output toward input_val by at most `slop` per call."""
    if input_val > output:
        output = min(input_val, output + slop)
    elif input_val < output:
        output = max(input_val, output - slop)
    else:
        output = input_val
    return output


def constrain(value, low, high):
    """Clamp value between low and high."""
    if value < low:
        return low
    if value > high:
        return high
    return value


BANNER = """
╔══════════════════════════════════════════════════════════╗
║        Straight Line Assistant  +  TurtleBot3 Teleop     ║
╠══════════════════════════════════════════════════════════╣
║                                                          ║
║   Control Your TurtleBot3!                               ║
║   ──────────────────────                                 ║
║   Moving around:                                         ║
║           w                                              ║
║      a    s    d                                         ║
║           x                                              ║
║                                                          ║
║   w/x : increase/decrease linear velocity                ║
║   a/d : increase/decrease angular velocity               ║
║   space / s : force stop                                 ║
║                                                          ║
║   CTRL-C to quit                                         ║
║                                                          ║
║   Heading-hold PID engages automatically when driving    ║
║   straight (angular velocity ≈ 0).                       ║
║                                                          ║
╚══════════════════════════════════════════════════════════╝
"""


class StraightLineAssistant(Node):
    def __init__(self):
        super().__init__('straight_line_assistant')

        # ── Determine TurtleBot3 model limits ────────────────────────
        model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
        if model == 'burger':
            self.max_lin_vel = BURGER_MAX_LIN_VEL
            self.max_ang_vel = BURGER_MAX_ANG_VEL
        else:
            self.max_lin_vel = WAFFLE_MAX_LIN_VEL
            self.max_ang_vel = WAFFLE_MAX_ANG_VEL

        self.get_logger().info(
            f'TurtleBot3 model: {model}  '
            f'(max_lin={self.max_lin_vel}, max_ang={self.max_ang_vel})')

        # ── Teleop state (accumulated velocities, TurtleBot3 style) ──
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0   # smoothed output
        self.control_angular_vel = 0.0  # smoothed output

        # ── PID state ────────────────────────────────────────────────
        self.current_yaw = 0.0
        self.target_yaw = None
        self.is_going_straight = False
        self.integral = 0.0
        self.prev_error = 0.0

        # ── Declare parameters ───────────────────────────────────────
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('integral_limit', 1.0)   # anti-windup clamp
        self.declare_parameter('angular_epsilon', 1e-3)  # "no turn" threshold
        self.declare_parameter('key_timeout', 0.6)       # seconds idle → stop
        self.declare_parameter('odom_timeout', 0.5)      # stale odom threshold
        self.declare_parameter('control_rate', 20.0)     # Hz
        self.declare_parameter('odom_topic', '/odometry/filtered')     # or '/odom'

        # ── Read parameters ──────────────────────────────────────────
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.angular_epsilon = self.get_parameter('angular_epsilon').value
        self.key_timeout = self.get_parameter('key_timeout').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        control_rate = self.get_parameter('control_rate').value
        odom_topic = self.get_parameter('odom_topic').value

        # ── Subscriber ───────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10)
        self.get_logger().info(f'Subscribing to odometry on: {odom_topic}')

        # ── Publishers ───────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID debug topic — graph with:
        #   rqt_plot /pid_debug/data[0]   # yaw error (rad)
        #   rqt_plot /pid_debug/data[1]   # P term
        #   rqt_plot /pid_debug/data[2]   # I term
        #   rqt_plot /pid_debug/data[3]   # D term
        #   rqt_plot /pid_debug/data[4]   # total correction (angular.z out)
        #   rqt_plot /pid_debug/data[5]   # target yaw (rad)
        #   rqt_plot /pid_debug/data[6]   # current yaw (rad)
        self.pid_pub = self.create_publisher(
            Float32MultiArray, '/pid_debug', 10)

        # ── Timing ───────────────────────────────────────────────────
        now = self.get_clock().now()
        self.last_key_time = now
        self.last_odom_time = None
        self.last_control_time = now
        self.dt = 1.0 / control_rate
        self.timer = self.create_timer(self.dt, self.control_loop)

        # ── Keyboard thread ──────────────────────────────────────────
        self.running = True
        self.old_terminal_settings = None
        if os.name != 'nt' and sys.stdin.isatty():
            self.old_terminal_settings = termios.tcgetattr(sys.stdin)
            self.key_thread = threading.Thread(
                target=self._key_loop, daemon=True)
            self.key_thread.start()
        else:
            self.get_logger().warn(
                'stdin is not a terminal — keyboard input disabled. '
                'Run with: ros2 run straight_line_assistant '
                'straight_line_assistant_node')

        # ── Show banner ──────────────────────────────────────────────
        print(BANNER)
        self._print_vel()

    # ─────────────────────────────────────────────────────────────────
    # Keyboard handling (runs in a daemon thread)
    # ─────────────────────────────────────────────────────────────────
    def _read_key(self, timeout=0.1):
        """Read a single character from stdin with a timeout.
        Returns '' if nothing is available within the timeout."""
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return ''

    def _key_loop(self):
        """Background thread: read keys → update target velocities
        using TurtleBot3-style incremental w/a/s/d/x controls."""
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                key = self._read_key(timeout=0.1)
                if not key:
                    continue

                # CTRL-C → graceful shutdown
                if key == '\x03':
                    self.running = False
                    self.target_linear_vel = 0.0
                    self.target_angular_vel = 0.0
                    self.control_linear_vel = 0.0
                    self.control_angular_vel = 0.0
                    self.is_going_straight = False
                    self.cmd_pub.publish(Twist())
                    break

                if key == 'w':
                    self.target_linear_vel = constrain(
                        self.target_linear_vel + LIN_VEL_STEP_SIZE,
                        -self.max_lin_vel, self.max_lin_vel)
                    self.last_key_time = self.get_clock().now()

                elif key == 'x':
                    self.target_linear_vel = constrain(
                        self.target_linear_vel - LIN_VEL_STEP_SIZE,
                        -self.max_lin_vel, self.max_lin_vel)
                    self.last_key_time = self.get_clock().now()

                elif key == 'a':
                    self.target_angular_vel = constrain(
                        self.target_angular_vel + ANG_VEL_STEP_SIZE,
                        -self.max_ang_vel, self.max_ang_vel)
                    self.last_key_time = self.get_clock().now()

                elif key == 'd':
                    self.target_angular_vel = constrain(
                        self.target_angular_vel - ANG_VEL_STEP_SIZE,
                        -self.max_ang_vel, self.max_ang_vel)
                    self.last_key_time = self.get_clock().now()

                elif key == 's' or key == ' ':
                    # Force stop
                    self.target_linear_vel = 0.0
                    self.target_angular_vel = 0.0
                    self.control_linear_vel = 0.0
                    self.control_angular_vel = 0.0
                    self.is_going_straight = False
                    self.target_yaw = None
                    self.integral = 0.0
                    self.prev_error = 0.0
                    self.last_key_time = self.get_clock().now()

                else:
                    # Unknown key — ignore
                    continue

                # ── Update heading-hold state ────────────────────────
                if (abs(self.target_linear_vel) > 0.0
                        and abs(self.target_angular_vel)
                        < self.angular_epsilon):
                    if not self.is_going_straight:
                        # Lock the current heading as target
                        self.target_yaw = self.current_yaw
                        self.is_going_straight = True
                        self.integral = 0.0
                        self.prev_error = 0.0
                else:
                    # User is intentionally turning, or stopped
                    self.is_going_straight = False
                    self.target_yaw = None
                    self.integral = 0.0
                    self.prev_error = 0.0

                self._print_vel()

        finally:
            # Always restore terminal settings
            if self.old_terminal_settings is not None:
                try:
                    termios.tcsetattr(
                        sys.stdin, termios.TCSADRAIN,
                        self.old_terminal_settings)
                except Exception:
                    pass

    def _print_vel(self):
        """Overwrite the status line in the terminal."""
        hold = 'ON ' if self.is_going_straight else 'OFF'
        line = (
            f'\r  lin_vel {self.target_linear_vel:+.2f} / '
            f'{self.max_lin_vel:.2f}  |  '
            f'ang_vel {self.target_angular_vel:+.2f} / '
            f'{self.max_ang_vel:.2f}  |  '
            f'heading-hold {hold}   '
        )
        sys.stdout.write(line)
        sys.stdout.flush()

    # ─────────────────────────────────────────────────────────────────
    # Odometry callback
    # ─────────────────────────────────────────────────────────────────
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion(q)
        self.last_odom_time = self.get_clock().now()

    # ─────────────────────────────────────────────────────────────────
    # Angle normalisation helper
    # ─────────────────────────────────────────────────────────────────
    def normalize_angle(self, angle):
        """Keep angle between -π and π."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ─────────────────────────────────────────────────────────────────
    # 20 Hz control loop (timer callback — always fires)
    # ─────────────────────────────────────────────────────────────────
    def control_loop(self):
        now = self.get_clock().now()

        # Actual elapsed time (handles timer jitter)
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        if dt <= 0.0:
            dt = self.dt

        # ── Smooth velocity ramping (TurtleBot3-style) ───────────────
        # No key timeout watchdog — TurtleBot3-style controls persist
        # velocity until the user explicitly stops with s/space.
        self.control_linear_vel = make_simple_profile(
            self.control_linear_vel, self.target_linear_vel,
            LIN_VEL_STEP_SIZE / 2.0)
        self.control_angular_vel = make_simple_profile(
            self.control_angular_vel, self.target_angular_vel,
            ANG_VEL_STEP_SIZE / 2.0)

        # ── Build output message ─────────────────────────────────────
        out_msg = Twist()
        out_msg.linear.x = self.control_linear_vel

        # Check whether odometry is fresh enough to steer with
        odom_stale = (
            self.last_odom_time is None
            or (now - self.last_odom_time).nanoseconds / 1e9
            > self.odom_timeout
        )

        if (self.is_going_straight
                and self.target_yaw is not None
                and not odom_stale):
            # ── PID heading correction ───────────────────────────────
            error = self.normalize_angle(self.target_yaw - self.current_yaw)

            self.integral += error * dt
            self.integral = max(
                -self.integral_limit,
                min(self.integral_limit, self.integral))

            derivative = (
                (error - self.prev_error) / dt if dt > 0.0 else 0.0)

            pid_correction = (
                (self.kp * error)
                + (self.ki * self.integral)
                + (self.kd * derivative))
            pid_correction = max(
                -self.max_ang_vel,
                min(self.max_ang_vel, pid_correction))

            out_msg.angular.z = pid_correction
            self.prev_error = error

            p_term = self.kp * error
            i_term = self.ki * self.integral
            d_term = self.kd * derivative
            self.get_logger().debug(
                f'yaw_err={error:.4f}  P={p_term:.4f}  '
                f'I={i_term:.4f}  D={d_term:.4f}  '
                f'out={pid_correction:.4f}')

            # Publish all signals for rqt_plot
            dbg = Float32MultiArray()
            dbg.data = [
                float(error),           # [0] yaw error
                float(p_term),          # [1] P term
                float(i_term),          # [2] I term
                float(d_term),          # [3] D term
                float(pid_correction),  # [4] angular.z output
                float(self.target_yaw), # [5] target yaw
                float(self.current_yaw) # [6] current yaw
            ]
            self.pid_pub.publish(dbg)
        else:
            # Intentional turn, stopped, or stale odom → pass through
            out_msg.angular.z = self.control_angular_vel

        self.cmd_pub.publish(out_msg)

    # ─────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────
    def destroy_node(self):
        """Ensure we send a stop and restore the terminal."""
        self.running = False
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        if self.old_terminal_settings is not None:
            try:
                termios.tcsetattr(
                    sys.stdin, termios.TCSADRAIN,
                    self.old_terminal_settings)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineAssistant()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
