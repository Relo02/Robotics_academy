# Section 3 — Robot Control with ros2_control

## Overview

This section introduces the **ros2_control** framework, the standard way to manage hardware controllers in ROS 2. You will learn how to configure controller pipelines, spawn joint state broadcasters and velocity controllers, and send motion commands to the simulated bumperbot.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_description/      # Robot model with ros2_control hardware interface
├── bumperbot_controller/       # Controller configuration and launch files
├── bumperbot_cpp_examples/     # C++ velocity command publisher
└── bumperbot_py_examples/      # Python velocity command publisher
```

## Packages

### `bumperbot_controller`
Central package for the robot's control stack.

| File | Description |
|------|-------------|
| `config/bumperbot_controllers.yaml` | Defines controller types and parameters (joint names, PID gains) |
| `config/joy_config.yaml` | Joystick axis/button mappings |
| `config/joy_teleop.yaml` | Maps joystick axes to velocity commands |
| `launch/controller.launch.py` | Launches controller_manager and spawns all controllers |
| `launch/joystick_teleop.launch.py` | Launches joystick teleoperation stack |

### `bumperbot_description`
Updated with `bumperbot_ros2_control.xacro` to bind joints to the ros2_control hardware interface.

## Key ROS 2 Concepts

- **controller_manager**: central ROS 2 node that loads, configures, and manages controllers
- **hardware_interface**: abstraction layer between controllers and physical/simulated hardware
- **joint_state_broadcaster**: reads joint states from hardware and publishes to `/joint_states`
- **diff_drive_controller**: converts `Twist` velocity commands to individual wheel velocity targets
- **spawner**: tool to load and activate controllers at launch time

## Controller Pipeline

```
Twist command (/cmd_vel)
        │
        ▼
diff_drive_controller
        │
        ▼
hardware_interface (Gazebo simulation)
        │
        ▼
joint_state_broadcaster → /joint_states → robot_state_publisher → TF
```

## Running the Examples

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

### Launch the full control stack

```bash
ros2 launch bumperbot_controller controller.launch.py
```

### Send a velocity command manually

```bash
ros2 topic pub /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{ header: {stamp: now}, twist: { linear: { x: 0.2 }, angular: { z: 0.1 } } }"
```

### Joystick teleoperation

```bash
ros2 launch bumperbot_controller joystick_teleop.launch.py
```

### Inspect the controllers

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `ros2_control` | Core control framework (controller_manager, hardware_interface) |
| `ros2_controllers` | Standard controllers (diff_drive_controller, joint_state_broadcaster) |
| `joy` | Reads joystick input and publishes `sensor_msgs/Joy` |
| `joy_teleop` | Maps joystick axes to ROS 2 command messages |

## What You Will Learn

- How to configure and launch the ros2_control stack
- How the controller_manager orchestrates controllers and hardware interfaces
- How to use the differential drive controller for mobile robots
- How to teleoperate a robot with a joystick
- How to inspect and debug the control pipeline from the CLI