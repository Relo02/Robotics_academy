# Section 5 — Differential Drive Kinematics

## Overview

This section implements the **kinematic model of a differential drive robot**. You will write the forward and inverse kinematic equations that convert between the robot's linear/angular velocity and the individual wheel velocities — the mathematical core of any wheeled mobile robot controller.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_description/      # Robot model with wheel geometry
├── bumperbot_controller/       # Differential drive kinematic controller
├── bumperbot_cpp_examples/     # C++ kinematic examples
└── bumperbot_py_examples/      # Python kinematic examples
```

## Key Implementation — `simple_controller`

Located in `bumperbot_controller/src/simple_controller.cpp` and `bumperbot_controller/bumperbot_controller/simple_controller.py`.

### Robot Parameters

| Parameter | Symbol | Typical Value |
|-----------|--------|---------------|
| Wheel radius | $r$ | 0.033 m |
| Wheel separation | $l$ | 0.17 m |

### Forward Kinematics — wheel velocities → robot velocity

Given left and right wheel angular velocities $\omega_L$ and $\omega_R$:

$$v = \frac{r}{2}(\omega_R + \omega_L)$$

$$\omega = \frac{r}{l}(\omega_R - \omega_L)$$

### Inverse Kinematics — robot velocity → wheel velocities

Given desired robot linear velocity $v$ and angular velocity $\omega$:

$$\omega_R = \frac{v + \omega \, \frac{l}{2}}{r}$$

$$\omega_L = \frac{v - \omega \, \frac{l}{2}}{r}$$

### Matrix Formulation

Both equations are implemented as a single matrix multiplication using the Eigen library:

$$\begin{pmatrix} v \\ \omega \end{pmatrix} = \underbrace{\begin{pmatrix} \frac{r}{2} & \frac{r}{2} \\ \frac{r}{l} & -\frac{r}{l} \end{pmatrix}}_{M_{\text{speed}}} \begin{pmatrix} \omega_R \\ \omega_L \end{pmatrix}$$

The inverse (for the velocity command path) is computed as $M_{\text{speed}}^{-1}$.

### Node I/O

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/joint_states` | Subscribe | `sensor_msgs/JointState` | Wheel encoder positions |
| `/bumperbot_controller/cmd_vel` | Subscribe | `geometry_msgs/TwistStamped` | Desired robot velocity |
| `/simple_velocity_controller/commands` | Publish | `std_msgs/Float64MultiArray` | Target wheel velocities |

## Running the Examples

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# Launch the full simulation with differential drive controller
ros2 launch bumperbot_controller controller.launch.py

# Send a velocity command
ros2 topic pub /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{ header: {stamp: now}, twist: { linear: { x: 0.3 }, angular: { z: 0.5 } } }"

# Observe computed wheel velocities
ros2 topic echo /simple_velocity_controller/commands
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `ros2_control` | Hardware interface and controller manager |
| `ros2_controllers` | `joint_state_broadcaster`, velocity controllers |
| `eigen` (via ros-base) | Matrix algebra for the speed conversion matrix |
| `sensor_msgs` | `JointState` for wheel encoder data |
| `geometry_msgs` | `TwistStamped` for velocity commands |

## What You Will Learn

- The mathematical model of a differential drive robot
- How to translate kinematic equations into code using Eigen matrices
- How to interface a custom controller with the ros2_control framework
- How to subscribe to joint states and publish wheel commands
- Forward vs. inverse kinematics and when each is used
