# Section 9 — Sensor Fusion

## Overview

This section tackles the problem of **sensor fusion** — combining multiple noisy sensor measurements to produce a state estimate that is more accurate and robust than any single sensor alone. The key algorithm introduced is the **Extended Kalman Filter (EKF)**, implemented via the `robot_localization` package.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_msgs/             # Custom interfaces
├── bumperbot_description/      # Robot model (with IMU)
├── bumperbot_controller/       # Noisy wheel odometry (from Section 8)
├── bumperbot_localization/     # EKF node configuration
├── bumperbot_cpp_examples/     # C++ examples
└── bumperbot_py_examples/      # Python examples
```

## The Sensor Fusion Problem

Each sensor provides a measurement of the robot state with its own noise characteristics:

| Sensor | Measures | Noise type |
|--------|----------|------------|
| Wheel encoders | Linear velocity, angular velocity | Proportional to wheel slip |
| IMU (gyroscope) | Angular velocity | Gyro drift (bias) |
| IMU (accelerometer) | Linear acceleration | High-frequency noise |

Fusing these sources with a Kalman filter yields an estimate with lower variance than any individual sensor.

## The Extended Kalman Filter

The EKF operates in two alternating steps.

### Prediction Step

Using the motion model $f(\mathbf{x}, \mathbf{u})$ (e.g., odometry kinematics):

$$\hat{\mathbf{x}}_{k|k-1} = f(\mathbf{x}_{k-1}, \mathbf{u}_k)$$

$$P_{k|k-1} = F_k P_{k-1} F_k^T + Q_k$$

where $F_k = \frac{\partial f}{\partial \mathbf{x}}$ is the Jacobian, $P$ is the state covariance, and $Q$ is the process noise covariance.

### Update Step

When a sensor measurement $\mathbf{z}_k$ arrives:

$$K_k = P_{k|k-1} H_k^T \left(H_k P_{k|k-1} H_k^T + R_k\right)^{-1}$$

$$\mathbf{x}_k = \hat{\mathbf{x}}_{k|k-1} + K_k\left(\mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1})\right)$$

$$P_k = (I - K_k H_k) P_{k|k-1}$$

where $K_k$ is the **Kalman gain**, $H_k = \frac{\partial h}{\partial \mathbf{x}}$ is the measurement Jacobian, and $R_k$ is the measurement noise covariance.

The Kalman gain $K_k$ automatically weights measurements: a sensor with low $R$ (low noise) gets a higher weight; a sensor with high $P$ (high prediction uncertainty) causes the filter to trust measurements more.

## Key Package — `robot_localization`

The `bumperbot_localization` package configures `robot_localization`'s `ekf_node` to fuse wheel odometry and IMU data.

### Configuration (`ekf.yaml`)

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    odom0: /bumperbot_controller/odom          # wheel odometry
    odom0_config: [true, true, false,           # x, y, z
                   false, false, true,          # roll, pitch, yaw
                   true, true, false,           # vx, vy, vz
                   false, false, true]          # vroll, vpitch, vyaw
    imu0: /imu/data                             # IMU
    imu0_config: [false, false, false,
                  true, true, true,
                  false, false, false,
                  true, true, true,
                  true, true, true]
```

### Output

| Topic | Frame | Description |
|-------|-------|-------------|
| `/odometry/filtered` | `odom → base_footprint` | Fused pose estimate |

## Running the Examples

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# Launch simulation
ros2 launch bumperbot_description gazebo.launch.py

# Launch controller (noisy odometry)
ros2 launch bumperbot_controller controller.launch.py

# Launch EKF localization
ros2 launch bumperbot_localization localization.launch.py

# Compare raw noisy odometry vs filtered estimate
ros2 topic echo /bumperbot_controller/odom_noisy
ros2 topic echo /odometry/filtered

# Visualize in RViz
rviz2
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `robot_localization` | EKF and UKF state estimation nodes |
| `nav_msgs` | `Odometry` message type |
| `sensor_msgs` | `Imu` message type |
| `tf2_ros` | Publishing the fused TF transform |

## What You Will Learn

- The mathematical derivation of the Kalman filter prediction and update steps
- How the Extended Kalman Filter handles nonlinear motion and measurement models
- How to configure `robot_localization`'s EKF node to fuse multiple sensor streams
- How the Kalman gain automatically weights sensors by their noise levels
- How to validate sensor fusion by comparing filtered vs. raw odometry in RViz
