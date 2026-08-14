import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


def euler_from_quaternion(q):
    # Convert quaternion to euler yaw (Z-axis rotation)
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)


class StraightLineAssistant(Node):
    def __init__(self):
        super().__init__('straight_line_assistant')

        # --- State (initialized before subscriptions/timer so nothing can
        # reference it before it exists) ---
        self.teleop_cmd = Twist()
        self.current_yaw = 0.0
        self.target_yaw = None
        self.is_going_straight = False
        self.integral = 0.0
        self.prev_error = 0.0

        # --- Tunable parameters (use `ros2 param set` to retune live
        # instead of editing code and restarting) ---
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('integral_limit', 1.0)     # anti-windup clamp
        self.declare_parameter('max_angular_z', 1.5)       # rad/s, match your robot
        self.declare_parameter('angular_epsilon', 1e-3)    # "no intentional turn" threshold
        self.declare_parameter('teleop_timeout', 0.5)      # seconds
        self.declare_parameter('odom_timeout', 0.5)        # seconds

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.angular_epsilon = self.get_parameter('angular_epsilon').value
        self.teleop_timeout = self.get_parameter('teleop_timeout').value
        self.odom_timeout = self.get_parameter('odom_timeout').value

        # Subscribe to standard teleop output and filtered odometry
        self.cmd_sub = self.create_subscription(Twist, '/teleop/cmd_vel', self.teleop_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)

        # Publish corrected commands to the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Watchdog / real-dt bookkeeping
        now = self.get_clock().now()
        self.last_teleop_time = now
        self.last_odom_time = None
        self.last_control_time = now

        # 20 Hz Control Loop
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion(q)
        self.last_odom_time = self.get_clock().now()

    def teleop_cb(self, msg):
        self.teleop_cmd = msg
        self.last_teleop_time = self.get_clock().now()

        # If commanding forward/backward motion with zero intentional rotation
        if abs(msg.linear.x) > 0.0 and abs(msg.angular.z) < self.angular_epsilon:
            if not self.is_going_straight:
                # Lock the current heading as our target
                self.target_yaw = self.current_yaw
                self.is_going_straight = True
                self.integral = 0.0
                self.prev_error = 0.0
        else:
            self.is_going_straight = False

    def normalize_angle(self, angle):
        # Keep angle error between -pi and pi
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        now = self.get_clock().now()

        # Actual elapsed time since the last tick, rather than assuming a
        # perfectly periodic 0.05s — ROS2 timers can jitter under load.
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        if dt <= 0.0:
            dt = self.dt

        # Watchdog: if teleop has gone quiet, stop instead of replaying the
        # last command forever.
        teleop_elapsed = (now - self.last_teleop_time).nanoseconds / 1e9
        if teleop_elapsed > self.teleop_timeout:
            self.teleop_cmd = Twist()
            self.is_going_straight = False

        out_msg = Twist()
        out_msg.linear.x = self.teleop_cmd.linear.x

        # If odometry has gone stale, don't steer using frozen yaw data —
        # fall back to passing the raw command through.
        odom_stale = (
            self.last_odom_time is None
            or (now - self.last_odom_time).nanoseconds / 1e9 > self.odom_timeout
        )

        if self.is_going_straight and self.target_yaw is not None and not odom_stale:
            # PID Calculation
            error = self.normalize_angle(self.target_yaw - self.current_yaw)

            self.integral += error * dt
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
            derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0

            pid_correction = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            pid_correction = max(-self.max_angular_z, min(self.max_angular_z, pid_correction))

            out_msg.angular.z = pid_correction
            self.prev_error = error

            self.get_logger().debug(f'yaw_error={error:.3f} correction={pid_correction:.3f}')
        else:
            # User is intentionally turning, teleop is stale, or odom is
            # stale; pass commands through directly.
            out_msg.angular.z = self.teleop_cmd.angular.z

        self.cmd_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineAssistant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
