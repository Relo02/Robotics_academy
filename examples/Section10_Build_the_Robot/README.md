# Section 10 — Building the Physical Robot

## Overview

This is the capstone section. All concepts from previous sections — URDF modeling, ros2_control, kinematics, odometry, TF2, sensor fusion — are integrated into a **complete software stack for a real physical robot**. The robot communicates with an Arduino microcontroller over a serial port, uses lifecycle nodes for safe startup/shutdown, and runs the full localization pipeline.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_msgs/             # Custom interfaces
├── bumperbot_description/      # Final robot URDF
├── bumperbot_controller/       # High-level velocity controller
├── bumperbot_firmware/         # Hardware interface + serial communication
├── bumperbot_bringup/          # Top-level launch files
├── bumperbot_localization/     # EKF localization node
├── bumperbot_cpp_examples/     # C++ lifecycle and serial examples
└── bumperbot_py_examples/      # Python examples
```

## Key Package — `bumperbot_firmware`

This package bridges ROS 2 and the physical hardware (Arduino).

### Hardware Interface — `bumperbot_interface`

Implements `hardware_interface::SystemInterface` from the ros2_control framework. This is the plugin that the `controller_manager` loads at runtime.

```
controller_manager
        │  (loads plugin via pluginlib)
        ▼
bumperbot_interface::BumperbotInterface
        │  (serial port: /dev/ttyUSB0)
        ▼
Arduino firmware
        │  (motor PWM + encoder feedback)
        ▼
Physical wheels
```

#### Lifecycle Callbacks

The hardware interface uses `rclcpp_lifecycle` to guarantee safe state transitions:

| Callback | Action |
|----------|--------|
| `on_init` | Parse URDF parameters (port, baud rate) |
| `on_configure` | Open the serial port |
| `on_activate` | Start sending commands and reading encoders |
| `on_deactivate` | Stop commands, keep port open |
| `on_cleanup` | Close the serial port |
| `on_shutdown` | Emergency stop |

### Serial Communication Nodes

| Node | File | Description |
|------|------|-------------|
| `simple_serial_receiver` | `src/simple_serial_receiver.cpp` | Reads data from serial port, publishes to ROS topic |
| `simple_serial_transmitter` | `src/simple_serial_transmitter.cpp` | Subscribes to a topic and writes to serial port |

The serial protocol uses `libserial` (C++) and `pyserial` (Python) to read encoder counts and write PWM commands as newline-delimited ASCII strings.

### `bumperbot_bringup`

Contains launch files that start the entire robot stack in the correct order:

```bash
# Simulation bringup
ros2 launch bumperbot_bringup simulated_robot.launch.py

# Real robot bringup
ros2 launch bumperbot_bringup real_robot.launch.py
```

### `bumperbot_localization`

Wraps `robot_localization`'s EKF node with robot-specific configuration, fusing wheel odometry and IMU data (same as Section 9 but tuned for real hardware).

## I2C Bus (Python — `python3-smbus`)

For robots equipped with I2C sensors (IMU, compass), the `bumperbot_py_examples` package includes I2C bus reading examples using `python3-smbus`.

## Serial Protocol Overview

The Arduino sends encoder tick counts over serial at a fixed rate:

```
"<left_ticks>,<right_ticks>\n"   (from Arduino → ROS)
"<left_pwm>,<right_pwm>\n"       (from ROS → Arduino)
```

The hardware interface converts between encoder ticks and radians using:

$$\phi = \frac{2\pi \cdot \text{ticks}}{\text{ticks\_per\_revolution}}$$

## Running on the Physical Robot

```bash
# 1. Build the workspace
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# 2. Check your serial port
ls /dev/ttyUSB*

# 3. Launch the real robot
ros2 launch bumperbot_bringup real_robot.launch.py

# 4. Teleoperate
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/bumperbot_controller/cmd_vel

# 5. Visualize in RViz
rviz2 -d ~/ros2_ws/src/bumperbot_description/rviz/display.rviz
```

## Running in Simulation

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

## Dependencies

| Package | Purpose |
|---------|---------|
| `ros2_control` | Hardware interface plugin system, controller_manager |
| `ros2_controllers` | Diff drive controller, joint state broadcaster |
| `pluginlib` | Runtime plugin loading for hardware interface |
| `rclcpp_lifecycle` | Lifecycle state machine for safe hardware management |
| `robot_localization` | EKF sensor fusion |
| `libserial-dev` | C++ serial port communication |
| `python3-serial` | Python serial port communication |
| `python3-smbus` | Python I2C bus access |

## What You Will Learn

- How to write a custom ros2_control hardware interface plugin
- How lifecycle nodes ensure safe hardware initialization and shutdown
- How to communicate with a microcontroller over a serial port
- How to design a complete robot software architecture
- How to bringup a physical robot system vs. a simulation
- How to integrate all previous course concepts into one cohesive stack
