"""
ekf_node.py — Extended Kalman Filter for differential-drive state estimation
=============================================================================

State vector:  x = [x, y, theta, v, omega]   (5-DOF)
               x, y      — position in the odom frame  [m]
               theta      — heading (yaw)                [rad]
               v          — linear  velocity            [m/s]
               omega      — angular velocity            [rad/s]

Inputs (fused):
  /odom          (nav_msgs/Odometry)   — wheel encoder odometry from diff_drive_controller
  /imu/data      (sensor_msgs/Imu)     — IMU linear acceleration + angular velocity

Output:
  /odometry/filtered  (nav_msgs/Odometry)   — EKF posterior pose + velocity
  /tf              odom → base_footprint    — broadcast for the rest of the TF tree

Algorithm
---------
1. Prediction step  (motion model, driven by IMU angular velocity + last velocity command):

       x_{k|k-1} = f(x_{k-1|k-1}, u_k)   where u_k = [v_cmd, omega_imu]

       f:
         x'     = x  + v * cos(theta) * dt
         y'     = y  + v * sin(theta) * dt
         theta' = theta + omega * dt
         v'     = v        (constant-velocity assumption; corrected by odom)
         omega' = omega    (constant, corrected by IMU)

       Jacobian F = ∂f/∂x  (analytical, see _jacobian_F)

       P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q

2. Update step  (measurement model for odometry):

       z  = [v_odom, omega_odom]    (linear and angular velocity from /odom)

       h(x) = [v, omega]           (identity mapping from state)

       Jacobian H:  5×2  selecting the v and omega rows

       Innovation:  y_inn = z - h(x_{k|k-1})
       Kalman gain: K = P_{k|k-1} * H^T * (H * P_{k|k-1} * H^T + R)^{-1}

       x_{k|k}   = x_{k|k-1} + K * y_inn
       P_{k|k}   = (I - K*H) * P_{k|k-1}

Noise matrices
--------------
  Q — process noise  (tunable via ROS parameters)
  R — measurement noise from wheel odometry  (tunable)

The IMU angular velocity is used directly in the prediction step (not as a
separate measurement update) to keep the implementation concise and readable.
For a production system consider a full EKF with a separate IMU update step.

Equivalent terminal commands (without launch file)
--------------------------------------------------
  ros2 run slam_ekf ekf_node \
      --ros-args \
      -p process_noise_v:=0.01 \
      -p process_noise_omega:=0.01 \
      -p measurement_noise_v:=0.05 \
      -p measurement_noise_omega:=0.05
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class EKFLocalizationNode(Node):
    """ROS 2 node that runs an EKF to fuse wheel odometry and IMU."""

    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__('ekf_localization')

        # ---- tuneable noise parameters (set via ros-args or launch) ----
        self.declare_parameter('process_noise_x',      0.01)
        self.declare_parameter('process_noise_y',      0.01)
        self.declare_parameter('process_noise_theta',  0.01)
        self.declare_parameter('process_noise_v',      0.05)
        self.declare_parameter('process_noise_omega',  0.05)
        self.declare_parameter('measurement_noise_v',  0.1)
        self.declare_parameter('measurement_noise_omega', 0.1)
        self.declare_parameter('publish_tf', True)

        # ---- EKF state: [x, y, theta, v, omega] ----------------------
        self._x = np.zeros(5)          # state mean
        self._P = np.eye(5) * 0.1      # state covariance

        # Process-noise covariance Q (diagonal)
        q_diag = [
            self.get_parameter('process_noise_x').value,
            self.get_parameter('process_noise_y').value,
            self.get_parameter('process_noise_theta').value,
            self.get_parameter('process_noise_v').value,
            self.get_parameter('process_noise_omega').value,
        ]
        self._Q = np.diag(q_diag)

        # Measurement-noise covariance R for [v, omega] from wheel odom
        r_diag = [
            self.get_parameter('measurement_noise_v').value,
            self.get_parameter('measurement_noise_omega').value,
        ]
        self._R = np.diag(r_diag)

        # Measurement Jacobian H  (2×5): selects v and omega from state
        self._H = np.zeros((2, 5))
        self._H[0, 3] = 1.0   # v
        self._H[1, 4] = 1.0   # omega

        # ---- timestamps -----------------------------------------------
        self._last_time = None   # rclpy.time.Time of last prediction
        self._imu_omega = 0.0   # latest angular velocity from IMU  [rad/s]

        # ---- TF broadcaster -------------------------------------------
        self._tf_br = TransformBroadcaster(self)

        # ---- subscribers ----------------------------------------------
        # /imu/data  →  use angular velocity z for prediction input
        # Equivalent: ros2 topic echo /imu/data
        self.create_subscription(Imu,       'imu/data',  self._imu_cb,  10)

        # /odom      →  wheel encoder odometry from diff_drive_controller
        # Equivalent: ros2 topic echo /odom
        self.create_subscription(Odometry,  'odom',      self._odom_cb, 10)

        # ---- publisher ------------------------------------------------
        # ros2 topic echo /odometry/filtered
        self._pub = self.create_publisher(Odometry, 'odometry/filtered', 10)

        self.get_logger().info('EKF localization node started.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _imu_cb(self, msg: Imu):
        """Cache the latest IMU angular velocity (used in prediction step)."""
        self._imu_omega = msg.angular_velocity.z

    def _odom_cb(self, msg: Odometry):
        """
        Main EKF loop: called every time a new wheel-odometry message arrives.

        Steps:
          1. Compute dt since the last update.
          2. Prediction step with motion model.
          3. Update step using wheel-odom velocity measurements.
          4. Publish filtered odometry and broadcast TF.
        """
        now = Time.from_msg(msg.header.stamp)

        # ---- compute dt -----------------------------------------------
        if self._last_time is None:
            self._last_time = now
            return
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            return   # skip degenerate intervals

        # ---- prediction step ------------------------------------------
        v_cmd     = msg.twist.twist.linear.x
        omega_imu = self._imu_omega
        self._predict(v_cmd, omega_imu, dt)

        # ---- update step ----------------------------------------------
        v_meas     = msg.twist.twist.linear.x
        omega_meas = msg.twist.twist.angular.z
        self._update(np.array([v_meas, omega_meas]))

        # ---- publish --------------------------------------------------
        self._publish_state(msg.header.stamp)

    # ------------------------------------------------------------------
    # EKF internals
    # ------------------------------------------------------------------

    def _predict(self, v: float, omega: float, dt: float):
        """
        Propagate state and covariance using the nonlinear motion model.

        x'     = x  + v * cos(theta) * dt
        y'     = y  + v * sin(theta) * dt
        theta' = theta + omega * dt
        v'     = v
        omega' = omega
        """
        x, y, th, _v, _w = self._x
        cos_th = math.cos(th)
        sin_th = math.sin(th)

        # Nonlinear state transition
        self._x[0] = x  + v * cos_th * dt
        self._x[1] = y  + v * sin_th * dt
        self._x[2] = th + omega * dt
        self._x[3] = v        # keep commanded velocity as prior
        self._x[4] = omega    # keep IMU omega as prior

        # Wrap theta to [-pi, pi]
        self._x[2] = math.atan2(math.sin(self._x[2]), math.cos(self._x[2]))

        # Analytical Jacobian  F = ∂f/∂x
        F = self._jacobian_F(v, th, dt)

        # Covariance propagation:  P = F P F^T + Q
        self._P = F @ self._P @ F.T + self._Q

    def _jacobian_F(self, v: float, theta: float, dt: float) -> np.ndarray:
        """
        Compute the 5×5 Jacobian of the motion model w.r.t. the state.

        Only non-trivial partial derivatives:
          ∂x'/∂theta     = -v * sin(theta) * dt
          ∂x'/∂v         =  cos(theta) * dt
          ∂y'/∂theta     =  v * cos(theta) * dt
          ∂y'/∂v         =  sin(theta) * dt
          ∂theta'/∂omega =  dt
        """
        F = np.eye(5)
        F[0, 2] = -v * math.sin(theta) * dt   # ∂x/∂theta
        F[0, 3] =      math.cos(theta) * dt   # ∂x/∂v
        F[1, 2] =  v * math.cos(theta) * dt   # ∂y/∂theta
        F[1, 3] =      math.sin(theta) * dt   # ∂y/∂v
        F[2, 4] =  dt                          # ∂theta/∂omega
        return F

    def _update(self, z: np.ndarray):
        """
        Standard EKF measurement update for z = [v_odom, omega_odom].

        Innovation:     y   = z - H x
        Kalman gain:    K   = P H^T (H P H^T + R)^{-1}
        State update:   x  += K y
        Cov update:     P   = (I - K H) P
        """
        H   = self._H
        y   = z - H @ self._x                          # innovation
        S   = H @ self._P @ H.T + self._R              # innovation covariance
        K   = self._P @ H.T @ np.linalg.inv(S)         # Kalman gain
        self._x = self._x + K @ y                      # state update
        self._P = (np.eye(5) - K @ H) @ self._P        # covariance update
        self._x[2] = math.atan2(math.sin(self._x[2]),
                                math.cos(self._x[2]))  # wrap theta

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_state(self, stamp):
        """Publish the EKF posterior as nav_msgs/Odometry and broadcast TF."""
        x, y, th, v, w = self._x

        # ---- Odometry message -----------------------------------------
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_footprint'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(th / 2.0)
        msg.pose.pose.orientation.w = math.cos(th / 2.0)

        msg.twist.twist.linear.x  = v
        msg.twist.twist.angular.z = w

        # Copy covariance (use diagonal of P for the 6-DOF covariance)
        cov = list(msg.pose.covariance)
        cov[0]  = self._P[0, 0]   # x
        cov[7]  = self._P[1, 1]   # y
        cov[35] = self._P[2, 2]   # yaw
        msg.pose.covariance = cov

        self._pub.publish(msg)

        # ---- TF broadcast  odom → base_footprint ----------------------
        if self.get_parameter('publish_tf').value:
            tf_msg = TransformStamped()
            tf_msg.header.stamp    = stamp
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id  = 'base_footprint'
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.rotation.z    = math.sin(th / 2.0)
            tf_msg.transform.rotation.w    = math.cos(th / 2.0)
            self._tf_br.sendTransform(tf_msg)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
