# Chapter 7 - Integrating `ros2_control` for Real-Time Actuation

> _Goal_: attach a velocity controller to the differential-drive wheels so you can command motion with standard ROS 2 interfaces in Gazebo Sim.

## 1 What Is `ros2_control`?

`ros2_control` is a hardware abstraction layer and controller framework. It exposes joints through command and state interfaces such as `position`, `velocity`, and `effort`, and lets controllers operate on top of those interfaces.

In simulation, Gazebo loads a control plugin that acts like the hardware layer. Your URDF declares which joints are controllable and which interfaces they export. A YAML file defines the controllers that run on top of those joints.

Outcome of this chapter:

1. Extend the URDF with `ros2_control` tags and a Gazebo control plugin.
2. Create a `racademy_controller` package containing a wheel velocity controller.
3. Spawn the controllers and command the robot with standard ROS topics.

---

## 2 Updating the Xacro Files

### 2.1 `racademy.gazebo.xacro`

The Gazebo plugin can live in the Gazebo-specific xacro file. Add a control plugin block there.

For a simple Gazebo Sim setup:

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find racademy_controller)/config/racademy_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

If you want to keep compatibility with older Humble setups that still use `ign_ros2_control`, wrap the plugin section in xacro conditionals:

```xml
<xacro:if value="$(arg is_ignition)">
  <gazebo>
    <plugin filename="ign_ros2_control-system"
            name="ign_ros2_control::IgnitionROS2ControlPlugin">
      <parameters>$(find racademy_controller)/config/racademy_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</xacro:if>

<xacro:unless value="$(arg is_ignition)">
  <gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find racademy_controller)/config/racademy_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</xacro:unless>
```

- `is_ignition` is useful if you want one tutorial to cover both older and newer ROS 2 distributions.
- The `<parameters>` tag points to the controller YAML file created in Section 3.

### 2.2 `racademy.ros2_control.xacro`

Create a new file:

`racademy_ws/src/racademy_description/urdf/racademy.ros2_control.xacro`

This file declares the controllable joints and their interfaces:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <ros2_control name="RobotSystem" type="system">

    <joint name="wheel_left_joint">
      <command_interface name="velocity">
        <param name="min">-10.0</param>
        <param name="max">10.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="wheel_right_joint">
      <command_interface name="velocity">
        <param name="min">-10.0</param>
        <param name="max">10.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

  </ros2_control>

</robot>
```

Why velocity?

A differential-drive robot is most naturally commanded through wheel angular velocity. Later you can place a `diff_drive_controller` on top of these wheel interfaces and expose `/cmd_vel`.

### 2.3 Include Blocks in the Main URDF

At the top of `racademy.urdf.xacro`, add:

```xml
<xacro:arg name="is_ignition" default="false"/>
<xacro:include filename="$(find racademy_description)/urdf/racademy.gazebo.xacro"/>
<xacro:include filename="$(find racademy_description)/urdf/racademy.ros2_control.xacro"/>
```

You already include `racademy.gazebo.xacro` in the current file, so the practical change is:

- add the `is_ignition` argument if you want the conditional plugin version
- add the `racademy.ros2_control.xacro` include

The wheel joints themselves already exist in the robot:

- `wheel_left_joint`
- `wheel_right_joint`

so no mechanical changes are required.

---

## 3 Creating `racademy_controller` Package

Create the package:

```bash
cd /github/racademy_ws/src
ros2 pkg create --build-type ament_cmake racademy_controller
```

### 3.1 YAML Configuration

Create:

`racademy_controller/config/racademy_controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    simple_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController
      joints:
        - wheel_left_joint
        - wheel_right_joint
```

This gives you:

- `joint_state_broadcaster` to publish wheel joint states
- `simple_velocity_controller` to command both wheels directly

### 3.2 Package Installation Rules

Edit `racademy_controller/CMakeLists.txt`:

```cmake
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

Edit `racademy_controller/package.xml` and add at least:

```xml
<exec_depend>ros2launch</exec_depend>
<exec_depend>controller_manager</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>velocity_controllers</exec_depend>
```

Those are the runtime pieces used by the launch file and controller spawners.

---

## 4 Controller Launch File

Create:

`racademy_controller/launch/controller.launch.py`

```python
#!/usr/bin/env python3
"""Spawn the wheel controllers for racademy."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    velocity_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([broadcaster, velocity_ctrl])
```

This launch file assumes Gazebo already loaded the `ros2_control` plugin from the robot description.

---

## 5 Installing Dependencies

For Gazebo Sim on newer ROS 2 distributions, install:

```bash
sudo apt install ros-$ROS_DISTRO-gz-ros2-control
```

For older Humble setups that still use the ignition plugin, the equivalent package is:

```bash
sudo apt install ros-humble-ign-ros2-control
```

If the control plugin fails to load, check that your simulator and `ros_gz` packages match your ROS distro.

---

## 6 Build and Run

Build the workspace:

```bash
cd /github/racademy_ws
colcon build --symlink-install
source install/setup.bash
```

Open three terminals and source each one.

| Terminal | Command |
| --- | --- |
| 1 | `ros2 launch racademy_description gazebo.launch.py` |
| 2 | `ros2 launch racademy_controller controller.launch.py` |
| 3 | Testing commands below |

### 6.1 Verify Controllers

```bash
ros2 control list_controllers
```

Expected:

```text
joint_state_broadcaster [active]
simple_velocity_controller [active]
```

### 6.2 Drive the Robot

Publish wheel velocities:

```bash
ros2 topic pub /simple_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [1.0, 1.0]}"
```

Examples:

- `[1.0, 1.0]` -> drive forward
- `[1.0, -1.0]` -> spin in place
- `[1.0, 0.0]` -> turn around one wheel

### 6.3 Visualize the Graph

```bash
ros2 run rqt_graph rqt_graph
```

You should see the controller topics connected to `controller_manager` and the Gazebo control plugin.

---

## 7 Next Steps

- Replace the simple wheel controller with `diff_drive_controller` so the robot accepts `/cmd_vel`.
- Add PID gains and tune them for smoother wheel tracking.
- Spawn the controllers from the main Gazebo launch file so everything starts in one command.

- **End of Chapter 7**
