#!/usr/bin/env python3
"""
EKF-based state estimator for the FWS robot.

State vector: [x, y, vx, vy, yaw]

Prediction:  IMU @ 100 Hz  (body-frame linear acceleration + yaw rate)
Correction:  2-D LiDAR @ 30 Hz  (point-to-point ICP scan matching -> accumulated pose)

ICP transforms consecutive scans to estimate the relative displacement
(dx, dy, d_yaw), which is accumulated into an absolute scan-matching pose
that serves as the EKF position/heading measurement.
"""

from typing import Optional, Tuple

import numpy as np
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class EKFStateEstimator(Node):
    """Extended Kalman Filter fusing IMU and 2-D LiDAR scan matching."""

    # State indices
    IX, IY, IVX, IVY, IYAW = 0, 1, 2, 3, 4
    N = 5

    def __init__(self):
        super().__init__('ekf_state_estimator')

        # ── State & covariance ──────────────────────────────────────────
        self._x = np.zeros(self.N)      # [x, y, vx, vy, yaw]
        self._P = np.eye(self.N) * 0.1

        # ── Process noise Q ─────────────────────────────────────────────
        # Tuned for the robot's IMU noise (stddev: accel 1.7e-2, gyro 2e-4)
        self._Q = np.diag([1e-2, 1e-2, 5e-3, 5e-3, 1e-4])

        # ── Scan-matching measurement noise R ───────────────────────────
        # z = [x, y, yaw]
        self._R_scan = np.diag([0.05, 0.05, 0.02])

        # ── IMU bookkeeping ─────────────────────────────────────────────
        self._last_imu_t: Optional[float] = None
        self._ax_body = 0.0
        self._ay_body = 0.0
        self._wz = 0.0

        # ── Scan-matching bookkeeping ───────────────────────────────────
        self._prev_pts: Optional[np.ndarray] = None
        self._sm_pose = np.zeros(3)     # accumulated [x, y, yaw]

        # ── ROS I/O ─────────────────────────────────────────────────────
        self._imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_cb, 10)
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)

        self._odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        self._tf_br = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('EKF state estimator started.')

    # ================================================================== #
    #  IMU callback — prediction step
    # ================================================================== #
    def _imu_cb(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._last_imu_t is None:
            self._last_imu_t = t
            return

        dt = t - self._last_imu_t
        self._last_imu_t = t

        if dt <= 0.0 or dt > 0.5:
            return

        self._ax_body = msg.linear_acceleration.x
        self._ay_body = msg.linear_acceleration.y
        self._wz = msg.angular_velocity.z

        self._predict(dt)
        self._publish(msg.header.stamp)

    def _predict(self, dt: float) -> None:
        """EKF prediction using IMU measurements as process inputs."""
        x, y, vx, vy, yaw = self._x
        ax_b, ay_b, wz = self._ax_body, self._ay_body, self._wz

        c, s = np.cos(yaw), np.sin(yaw)

        # Rotate body-frame acceleration to world frame
        ax_w = ax_b * c - ay_b * s
        ay_w = ax_b * s + ay_b * c

        # Propagate state (Euler integration)
        self._x[self.IX]   = x  + vx * dt
        self._x[self.IY]   = y  + vy * dt
        self._x[self.IVX]  = vx + ax_w * dt
        self._x[self.IVY]  = vy + ay_w * dt
        self._x[self.IYAW] = _wrap_angle(yaw + wz * dt)

        # Jacobian F = df/dx  (linearised around current state)
        F = np.eye(self.N)
        F[self.IX,   self.IVX]  = dt
        F[self.IY,   self.IVY]  = dt
        # d(ax_w)/d(yaw) = -ax_b*sin(yaw) - ay_b*cos(yaw)
        F[self.IVX,  self.IYAW] = (-ax_b * s - ay_b * c) * dt
        # d(ay_w)/d(yaw) =  ax_b*cos(yaw) - ay_b*sin(yaw)
        F[self.IVY,  self.IYAW] = ( ax_b * c - ay_b * s) * dt

        self._P = F @ self._P @ F.T + self._Q

    # ================================================================== #
    #  LiDAR callback — correction step
    # ================================================================== #
    def _scan_cb(self, msg: LaserScan) -> None:
        pts = self._scan_to_pts(msg)
        if pts is None:
            return

        pts = self._voxel_downsample(pts, voxel=0.05)
        if len(pts) < 20:
            return

        if self._prev_pts is None:
            self._prev_pts = pts
            return

        # ICP: estimate relative motion (source = prev scan, target = current)
        dx, dy, dyaw = self._icp(self._prev_pts, pts)
        self._prev_pts = pts

        # Accumulate relative transform into world-frame scan-matching pose
        cx, cy, cyaw = self._sm_pose
        self._sm_pose[0] = cx + dx * np.cos(cyaw) - dy * np.sin(cyaw)
        self._sm_pose[1] = cy + dx * np.sin(cyaw) + dy * np.cos(cyaw)
        self._sm_pose[2] = _wrap_angle(cyaw + dyaw)

        self._update_scan()

    def _update_scan(self) -> None:
        """EKF correction using accumulated scan-matching pose as measurement."""
        z = self._sm_pose.copy()    # [x, y, yaw]

        # H maps state [x, y, vx, vy, yaw] -> measurement [x, y, yaw]
        H = np.zeros((3, self.N))
        H[0, self.IX]   = 1.0
        H[1, self.IY]   = 1.0
        H[2, self.IYAW] = 1.0

        innov = z - H @ self._x
        innov[2] = _wrap_angle(innov[2])    # normalise heading innovation

        S = H @ self._P @ H.T + self._R_scan
        K = self._P @ H.T @ np.linalg.inv(S)

        self._x += K @ innov
        self._x[self.IYAW] = _wrap_angle(self._x[self.IYAW])
        self._P = (np.eye(self.N) - K @ H) @ self._P

    # ================================================================== #
    #  Helpers
    # ================================================================== #
    @staticmethod
    def _scan_to_pts(msg: LaserScan) -> Optional[np.ndarray]:
        """Convert LaserScan to an (N, 2) array of valid Cartesian points."""
        angles = np.linspace(msg.angle_min, msg.angle_max,
                             len(msg.ranges), endpoint=True)
        r = np.asarray(msg.ranges, dtype=np.float32)
        valid = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        if valid.sum() < 20:
            return None
        return np.column_stack([
            r[valid] * np.cos(angles[valid]),
            r[valid] * np.sin(angles[valid]),
        ])

    @staticmethod
    def _voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
        """Keep one representative point per voxel cell."""
        keys = np.floor(pts / voxel).astype(np.int32)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[idx]

    @staticmethod
    def _icp(source: np.ndarray,
             target: np.ndarray,
             max_iter: int = 30,
             tol: float = 1e-4) -> Tuple[float, float, float]:
        """
        Point-to-point 2-D ICP (SVD closed-form).

        Returns (dx, dy, dyaw): rigid transform mapping source -> target,
        expressed in the source frame.
        """
        src = source.copy()
        R_acc = np.eye(2)
        t_acc = np.zeros(2)
        tree = KDTree(target)

        for _ in range(max_iter):
            _, idx = tree.query(src, k=1)
            tgt_matched = target[idx]

            mu_s = src.mean(axis=0)
            mu_t = tgt_matched.mean(axis=0)
            H_mat = (src - mu_s).T @ (tgt_matched - mu_t)

            U, _, Vt = np.linalg.svd(H_mat)
            R = Vt.T @ U.T
            # Enforce proper rotation (det = +1, not a reflection)
            if np.linalg.det(R) < 0:
                Vt[-1] *= -1
                R = Vt.T @ U.T
            t = mu_t - R @ mu_s

            src = (R @ src.T).T + t
            R_acc = R @ R_acc
            t_acc = R @ t_acc + t

            step_angle = abs(np.arctan2(R[1, 0], R[0, 0]))
            if np.linalg.norm(t) < tol and step_angle < tol:
                break

        return float(t_acc[0]), float(t_acc[1]), float(np.arctan2(R_acc[1, 0], R_acc[0, 0]))

    def _publish(self, stamp) -> None:
        """Publish /ekf_odom and broadcast odom -> base_link TF."""
        x, y, vx, vy, yaw = self._x
        qz = float(np.sin(yaw / 2.0))
        qw = float(np.cos(yaw / 2.0))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(self._wz)
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = EKFStateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
