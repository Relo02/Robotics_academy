# Section 7 — Odometry

## Overview

This section implements **wheel odometry** — the process of estimating a robot's position and orientation over time by integrating wheel encoder measurements. It builds directly on the differential drive kinematics from Section 5 and the TF2 knowledge from Section 6.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_msgs/             # Custom service: GetTransform
├── bumperbot_description/      # Robot model
├── bumperbot_controller/       # Odometry controller
├── bumperbot_cpp_examples/     # C++ TF2 broadcaster
└── bumperbot_py_examples/      # Python TF2 broadcaster
```

## Key Implementation — `simple_controller` with Odometry

The controller in `bumperbot_controller/src/simple_controller.cpp` is extended to track the robot's pose.

### Odometry Integration

At each control step, the controller reads the wheel position deltas $\Delta\phi_L$ and $\Delta\phi_R$ and integrates:

$$\Delta s = \frac{r}{2}(\Delta\phi_R + \Delta\phi_L)$$

$$\Delta\theta = \frac{r}{l}(\Delta\phi_R - \Delta\phi_L)$$

The robot's pose in the `odom` frame is updated as:

$$x_{t+1} = x_t + \Delta s \cos\theta_t$$

$$y_{t+1} = y_t + \Delta s \sin\theta_t$$

$$\theta_{t+1} = \theta_t + \Delta\theta$$

### Velocities

Linear and angular velocities are estimated from positional deltas divided by the time step $\Delta t$:

$$v = \frac{\Delta s}{\Delta t}, \qquad \omega = \frac{\Delta\theta}{\Delta t}$$

### Outputs

| Output | Topic / Frame | Type | Description |
|--------|---------------|------|-------------|
| Odometry message | `/bumperbot_controller/odom` | `nav_msgs/Odometry` | Pose estimate with covariance |
| TF broadcast | `odom → base_footprint` | `geometry_msgs/TransformStamped` | Odometry transform for TF tree |

## Odometry Message Structure

The `nav_msgs/Odometry` message contains:

- **`pose.pose`** — estimated position $(x, y, z)$ and orientation (quaternion)
- **`twist.twist`** — estimated linear and angular velocity
- **`pose.covariance`** — $6 \times 6$ uncertainty matrix for pose
- **`twist.covariance`** — $6 \times 6$ uncertainty matrix for velocity

## Running the Examples

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# Launch simulation + controller
ros2 launch bumperbot_description gazebo.launch.py
ros2 launch bumperbot_controller controller.launch.py

# Drive the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/bumperbot_controller/cmd_vel

# Observe the odometry estimate
ros2 topic echo /bumperbot_controller/odom

# Visualize in RViz (add Odometry and TF displays)
rviz2
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `nav_msgs` | `Odometry` message type |
| `tf2_ros` | `TransformBroadcaster` for `odom → base_footprint` |
| `tf_transformations` | Euler ↔ quaternion conversion |
| `sensor_msgs` | `JointState` for encoder readings |
| `geometry_msgs` | `TwistStamped`, `TransformStamped` |

## What You Will Learn

- How to implement dead reckoning from wheel encoders
- How to integrate incremental motion to produce a global pose estimate
- How to format and publish `nav_msgs/Odometry`
- How to broadcast the `odom → base_footprint` TF transform
- How odometry error accumulates over time (motivating Section 8 and 9)
