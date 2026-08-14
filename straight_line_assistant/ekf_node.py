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
    def __init__(self):
        super().__init__('ekf_custom_node')

        # ── State Vector: [x, y, theta, v, omega]^T ──────────────────
        self.x = np.zeros((5, 1))

        # ── State Covariance P ───────────────────────────────────────
        self.P = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])

        # ── Process Noise Covariance Q ───────────────────────────────
        self.Q = np.diag([0.01, 0.01, 0.02, 0.05, 0.05])

        # ── Measurement Noise Covariances R ──────────────────────────
        # Odom measurement: [v_odom, omega_odom]
        self.R_odom = np.diag([0.05, 0.25])  # Higher noise on odom omega due to wheel slip

        # IMU measurement: [theta_imu, omega_imu]
        self.R_imu = np.diag([0.05, 0.01])   # Low noise on IMU gyro omega

        self.last_time = self.get_clock().now()
        self.initialized = False

        # ── Subscribers & Publishers ────────────────────────────────
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.filtered_odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

        # TF Broadcaster for odom -> base_footprint / base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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

    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now()
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
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

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
        odom_msg.pose.covariance = cov.tolist()

        cov_twist = np.zeros(36)
        cov_twist[0] = self.P[3, 3]   # vx
        cov_twist[35] = self.P[4, 4]  # wz
        odom_msg.twist.covariance = cov_twist.tolist()

        self.filtered_odom_pub.publish(odom_msg)


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
