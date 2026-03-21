# Section 2 — Locomotion and Robot Simulation

## Overview

This section introduces the **bumperbot** — a differential-drive mobile robot. You will learn how to describe a robot using URDF/Xacro, spawn it in the Gazebo simulator, visualize it in RViz, and use ROS 2 parameters to configure nodes at runtime.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_description/      # Robot model (URDF/Xacro), launch files, meshes
├── bumperbot_cpp_examples/     # C++ nodes (publisher, parameter usage)
└── bumperbot_py_examples/      # Python nodes (publisher, parameter usage)
```

## Packages

### `bumperbot_description`
Contains the complete robot definition and simulation launch files.

| File | Description |
|------|-------------|
| `urdf/bumperbot.urdf.xacro` | Main robot model: links, joints, visual geometry, inertia |
| `urdf/bumperbot_gazebo.xacro` | Gazebo-specific plugins (differential drive, sensors) |
| `urdf/bumperbot_ros2_control.xacro` | ros2_control hardware interface bindings |
| `launch/gazebo.launch.py` | Spawns the robot in Gazebo Harmonic with ros_gz_bridge |
| `launch/display.launch.py` | Visualizes the robot in RViz with joint_state_publisher_gui |
| `meshes/` | STL mesh files for all robot links |

### `bumperbot_py_examples` / `bumperbot_cpp_examples`
Extend Section 1 with parameter demonstration nodes.

| Node | Description |
|------|-------------|
| `simple_parameter` | Declares and reads ROS 2 parameters at runtime |
| `simple_publisher` | Publishes velocity commands to the robot |

## Key ROS 2 Concepts

- **URDF/Xacro**: XML-based robot description format; Xacro adds macros and parameterization
- **robot_state_publisher**: broadcasts TF transforms from the URDF joint state
- **joint_state_publisher_gui**: interactive sliders for manual joint control
- **Gazebo Harmonic**: physics simulator; `ros_gz_sim` spawns robots, `ros_gz_bridge` relays topics
- **Parameters**: runtime configuration values declared with `declare_parameter` / `get_parameter`

## Running the Examples

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

### Visualize in RViz (no simulation)

```bash
ros2 launch bumperbot_description display.launch.py
```

### Simulate in Gazebo

```bash
ros2 launch bumperbot_description gazebo.launch.py
```

### Parameter node

```bash
ros2 run bumperbot_py_examples simple_parameter
# In another terminal:
ros2 param list
ros2 param get /simple_parameter wheel_radius
ros2 param set /simple_parameter wheel_radius 0.05
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `robot_state_publisher` | Publishes TF from URDF + JointState |
| `joint_state_publisher_gui` | GUI sliders for joint angles |
| `xacro` | Xacro macro processor for URDF |
| `rviz2` | 3D visualization |
| `ros_gz_sim` | Gazebo Harmonic ROS 2 bridge |
| `ros_gz_bridge` | Topic/service bridge between ROS 2 and Gazebo |

## What You Will Learn

- How to model a robot using URDF and Xacro
- How to spawn and control a robot in Gazebo Harmonic
- How to visualize robot state with RViz
- How to use ROS 2 parameters for runtime configuration
- How ROS 2 and Gazebo exchange data via `ros_gz_bridge`