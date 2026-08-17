#!/usr/bin/env python3
"""
Custom Extended Kalman Filter (EKF) Node for Sensor Fusion.

Fuses wheel odometry (/odom) and IMU (/imu) to produce /odometry/filtered.
Wheel odometry provides linear velocity and pose (susceptible to wheel slip).
IMU provides precise angular rate and orientation (susceptible to low-frequency drift).

State Vector X = [x, y, theta, v, omega]^T
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def euler_from_quaternion(q):
    """Extract yaw from geometry_msgs/Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_euler(roll, pitch, yaw):
    """Create geometry_msgs/Quaternion from euler angles (roll, pitch, yaw)."""
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class EKFNode(Node):
    # Factor by which the published yaw covariance is inflated while the IMU
    # is stale (yaw fusion lost -> heading estimate is much less trustworthy).
    STALE_IMU_YAW_COV_INFLATION = 10.0

    def __init__(self):
        super().__init__('ekf_custom_node')

        # ── State Vector: [x, y, theta, v, omega]^T ──────────────────
        self.x = np.zeros((5, 1))

        # ── State Covariance P ───────────────────────────────────────
        self.P = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])

        # ── Parameters ───────────────────────────────────────────────
        # publish_tf: TurtleBot3's OpenCR firmware already broadcasts
        # odom -> base_footprint, so keep this False unless that is disabled.
        self.publish_tf = self.declare_parameter('publish_tf', False).value
        self.sensor_timeout = self.declare_parameter('sensor_timeout', 0.5).value
        self.watchdog_rate = self.declare_parameter('watchdog_rate', 2.0).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.imu_topic = self.declare_parameter('imu_topic', '/imu').value

        q_x = self.declare_parameter('q_x', 0.01).value
        q_y = self.declare_parameter('q_y', 0.01).value
        q_theta = self.declare_parameter('q_theta', 0.02).value
        q_v = self.declare_parameter('q_v', 0.05).value
        q_omega = self.declare_parameter('q_omega', 0.05).value

        r_odom_v = self.declare_parameter('r_odom_v', 0.05).value
        r_odom_omega = self.declare_parameter('r_odom_omega', 0.25).value
        r_imu_theta = self.declare_parameter('r_imu_theta', 0.05).value
        r_imu_omega = self.declare_parameter('r_imu_omega', 0.01).value

        # ── Process Noise Covariance Q ───────────────────────────────
        self.Q = np.diag([q_x, q_y, q_theta, q_v, q_omega])

        # ── Measurement Noise Covariances R ──────────────────────────
        # Odom measurement: [v_odom, omega_odom]
        self.R_odom = np.diag([r_odom_v, r_odom_omega])  # Higher noise on odom omega due to wheel slip

        # IMU measurement: [theta_imu, omega_imu]
        self.R_imu = np.diag([r_imu_theta, r_imu_omega])  # Low noise on IMU gyro omega

        self.last_time = self.get_clock().now()
        self.initialized = False

        # ── Sensor freshness tracking (watchdog) ────────────────────
        self.last_odom_msg_time = None
        self.last_imu_msg_time = None
        self.odom_fresh = False
        self.imu_fresh = False

        # ── Subscribers & Publishers ────────────────────────────────
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.filtered_odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

        # TF Broadcaster for odom -> base_footprint (only used if publish_tf)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Watchdog checks sensor freshness independently of the callbacks.
        watchdog_period = 1.0 / max(self.watchdog_rate, 1e-3)
        self.watchdog_timer = self.create_timer(watchdog_period, self.watchdog_callback)

        self.get_logger().info('Custom EKF Node initialized and ready.')

    def predict(self, dt):
        if dt <= 0:
            return

        x, y, theta, v, omega = self.x.flatten()

        # Non-linear motion update
        self.x[0, 0] = x + v * math.cos(theta) * dt
        self.x[1, 0] = y + v * math.sin(theta) * dt
        self.x[2, 0] = normalize_angle(theta + omega * dt)
        self.x[3, 0] = v
        self.x[4, 0] = omega

        # Jacobian F
        F = np.eye(5)
        F[0, 2] = -v * math.sin(theta) * dt
        F[0, 3] = math.cos(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt
        F[1, 3] = math.sin(theta) * dt
        F[2, 4] = dt

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q * dt

    def _seconds_since(self, stamp):
        """Seconds elapsed since a receipt timestamp (inf if never received)."""
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _odom_is_fresh(self):
        return self._seconds_since(self.last_odom_msg_time) <= self.sensor_timeout

    def _imu_is_fresh(self):
        return self._seconds_since(self.last_imu_msg_time) <= self.sensor_timeout

    def watchdog_callback(self):
        """Detect fresh<->stale transitions and report each exactly once."""
        odom_fresh = self._odom_is_fresh()
        imu_fresh = self._imu_is_fresh()

        if odom_fresh != self.odom_fresh:
            self.odom_fresh = odom_fresh
            if odom_fresh:
                msg = '/odom is publishing again - resuming /odometry/filtered output.'
                self.get_logger().info(msg)
            else:
                msg = (f'/odom went stale (no messages for {self.sensor_timeout:.2f} s) - '
                       'stopping /odometry/filtered output.')
                self.get_logger().warn(msg)
            print(f'[EKF WATCHDOG] {msg}', flush=True)

        if imu_fresh != self.imu_fresh:
            self.imu_fresh = imu_fresh
            if imu_fresh:
                msg = '/imu is publishing again - yaw fusion restored.'
                self.get_logger().info(msg)
            else:
                msg = (f'/imu went stale (no messages for {self.sensor_timeout:.2f} s) - '
                       'degrading to odom-only, yaw fusion lost '
                       f'(published yaw covariance inflated {self.STALE_IMU_YAW_COV_INFLATION:.0f}x).')
                self.get_logger().warn(msg)
            print(f'[EKF WATCHDOG] {msg}', flush=True)

    def odom_callback(self, msg: Odometry):
        self.last_odom_msg_time = self.get_clock().now()
        current_time = self.last_odom_msg_time
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if not self.initialized:
            # Initialize position & orientation from first odom message
            yaw = euler_from_quaternion(msg.pose.pose.orientation)
            self.x[0, 0] = msg.pose.pose.position.x
            self.x[1, 0] = msg.pose.pose.position.y
            self.x[2, 0] = yaw
            self.initialized = True
            return

        # 1. Prediction step
        self.predict(dt)

        # 2. Measurement Update (Odom velocities: v, omega)
        z = np.array([[msg.twist.twist.linear.x],
                      [msg.twist.twist.angular.z]])

        H = np.zeros((2, 5))
        H[0, 3] = 1.0  # v
        H[1, 4] = 1.0  # omega

        y = z - H @ self.x  # Residual
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2, 0] = normalize_angle(self.x[2, 0])
        self.P = (np.eye(5) - K @ H) @ self.P

        # Publish filtered state
        self.publish_filtered_odom(msg.header.stamp)

    def imu_callback(self, msg: Imu):
        self.last_imu_msg_time = self.get_clock().now()
        if not self.initialized:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 0.001:
            self.predict(dt)
            self.last_time = current_time

        # IMU Measurement: [theta, omega]
        yaw = euler_from_quaternion(msg.orientation)
        omega = msg.angular_velocity.z

        z = np.array([[yaw],
                      [omega]])

        H = np.zeros((2, 5))
        H[0, 2] = 1.0  # theta
        H[1, 4] = 1.0  # omega

        y = z - H @ self.x
        y[0, 0] = normalize_angle(y[0, 0])  # Normalize yaw error

        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2, 0] = normalize_angle(self.x[2, 0])
        self.P = (np.eye(5) - K @ H) @ self.P

    def publish_filtered_odom(self, stamp):
        # If wheel odometry is stale, stop publishing entirely.
        if not self._odom_is_fresh():
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        x, y, theta, v, omega = self.x.flatten()

        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, theta)

        odom_msg.twist.twist.linear.x = float(v)
        odom_msg.twist.twist.angular.z = float(omega)

        # Pose & Twist Covariance
        cov = np.zeros(36)
        cov[0] = self.P[0, 0]   # x
        cov[7] = self.P[1, 1]   # y
        cov[35] = self.P[2, 2]  # yaw
        if not self._imu_is_fresh():
            # IMU stale: yaw fusion lost, advertise much lower yaw confidence.
            cov[35] *= self.STALE_IMU_YAW_COV_INFLATION
        odom_msg.pose.covariance = cov.tolist()

        cov_twist = np.zeros(36)
        cov_twist[0] = self.P[3, 3]   # vx
        cov_twist[35] = self.P[4, 4]  # wz
        odom_msg.twist.covariance = cov_twist.tolist()

        self.filtered_odom_pub.publish(odom_msg)

        # TurtleBot3's OpenCR firmware already broadcasts odom -> base_footprint;
        # only publish it ourselves when explicitly requested via publish_tf.
        if not self.publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
