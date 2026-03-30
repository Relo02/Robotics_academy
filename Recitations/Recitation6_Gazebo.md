# Chapter 6 – Bringing the Robot to Life in Gazebo (Ignition / Gazebo Sim)

> _Goal_: spawn **racademy** into Gazebo Sim using the `racademy_description` package, with a cleaner URDF split, reasonable inertias, simplified collision geometry, and ROS 2 simulation time.

## 1 Why Gazebo Sim?

Gazebo Sim is the physics simulator we use to move from a static URDF in RViz to a robot that can exist in a world, collide with the ground, and publish simulated time.

Outcome of this chapter:

1. Split Gazebo-specific settings out of the main URDF.
2. Add inertial blocks so the model is usable in simulation.
3. Replace expensive collision meshes with simpler shapes where appropriate.
4. Launch Gazebo Sim, publish `/robot_description`, spawn the robot, and bridge `/clock`.

## 2 Creating Supporting Xacro Files

Place these files in `racademy_ws/src/racademy_description/urdf/`:

- `materials.xacro`
- `macros.xacro`
- `racademy.gazebo.xacro`

This keeps the main robot file readable:

- `materials.xacro`: reusable URDF materials such as `black`, `green`, and `orange`
- `macros.xacro`: small inertia helpers like `box_inertia`, `cylinder_inertia`, and `sphere_inertia`
- `racademy.gazebo.xacro`: Gazebo-only tags such as visual materials and wheel contact parameters

Example Gazebo fragment:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <gazebo reference="left_wheel_link">
    <material>Gazebo/Black</material>
    <mu1>5.0</mu1>
    <mu2>5.0</mu2>
    <kp>1000000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
  </gazebo>

  <gazebo reference="right_wheel_link">
    <material>Gazebo/Black</material>
    <mu1>5.0</mu1>
    <mu2>5.0</mu2>
    <kp>1000000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
  </gazebo>

</robot>
```

Why this split helps:

- the main robot xacro stays focused on links and joints
- Gazebo parameters stay isolated and easy to tune later
- materials and inertia formulas can be reused cleanly

## 3 Extending the Main URDF

Open `racademy_ws/src/racademy_description/urdf/racademy.urdf.xacro`.

Three important updates were made.

### 3.1 Include the helper files

At the top of the file:

```xml
<xacro:include filename="$(find racademy_description)/urdf/materials.xacro"/>
<xacro:include filename="$(find racademy_description)/urdf/macros.xacro"/>
<xacro:include filename="$(find racademy_description)/urdf/racademy.gazebo.xacro"/>
```

### 3.2 Add inertial blocks

Every physical link now has an `<inertial>` section:

- `base_link`
- `left_wheel_link`
- `right_wheel_link`
- `computer_link`
- `camera_link`

Example:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="${wheel_mass}"/>
  <xacro:cylinder_inertia m="${wheel_mass}" r="${wheel_radius}" h="${wheel_width}"/>
</inertial>
```

This is enough to make the model physically valid for simulation without overcomplicating the chapter.

### 3.3 Simplify collision geometry

The visual meshes are kept for appearance, but some collision geometry is simplified:

- chassis: still uses the mesh collision
- wheels: use `<sphere radius="${wheel_radius}"/>`
- computer: uses a small box
- camera: uses a small box

Example wheel link:

```xml
<link name="left_wheel_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${wheel_mass}"/>
    <xacro:cylinder_inertia m="${wheel_mass}" r="${wheel_radius}" h="${wheel_width}"/>
  </inertial>

  <visual>
    <origin xyz="0.00001 -0.05957 0.0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://racademy_description/meshes/duckiebot_leftwheel.dae"/>
    </geometry>
    <material name="black"/>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="${wheel_radius}"/>
    </geometry>
  </collision>
</link>
```

Why simplify collisions:

- Gazebo contacts are more stable with primitives than with triangle meshes
- simulation is faster
- the wheels behave more predictably on the ground

## 4 Creating `gazebo.launch.py`

Place the file in `racademy_ws/src/racademy_description/launch/gazebo.launch.py`.

This launch file now works with the current package layout:

- package: `racademy_description`
- default model: `urdf/racademy.urdf.xacro`
- Gazebo resource paths: set from the package share parent
- robot spawns from the `robot_description` topic
- `/clock` is bridged back to ROS 2

Core ideas in the launch file:

```python
pkg_share = get_package_share_directory("racademy_description")
share_root = str(Path(pkg_share).parent)

robot_description = ParameterValue(
    Command([FindExecutable(name="xacro"), " ", model]),
    value_type=str,
)

rsp_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
)

gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    ),
    launch_arguments={"gz_args": ["-r -v 4 ", world]}.items(),
)

spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=["-topic", "robot_description", "-name", entity_name],
)

clock_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
)
```

### 4.1 Launch Arguments

The launch file exposes:

- `model`: xacro path, defaulting to `racademy.urdf.xacro`
- `world`: Gazebo world, default `empty.sdf`
- `entity_name`: spawned entity name, default `racademy`
- `use_sim_time`: default `true`

### 4.2 Major Sections Explained

| **Block** | **What it does** |
| --- | --- |
| `SetEnvironmentVariable` | Extends `IGN_GAZEBO_RESOURCE_PATH` and `GZ_SIM_RESOURCE_PATH` so Gazebo can find installed meshes and resources. |
| `robot_state_publisher` | Publishes TF and `/robot_description` from the xacro-expanded model. |
| `ros_gz_sim ... gz_sim.launch.py` | Starts Gazebo Sim with the selected world. |
| `ros_gz_sim create` | Spawns the robot from `/robot_description`. |
| `ros_gz_bridge parameter_bridge` | Bridges `/clock` so ROS nodes use simulation time. |
| `TimerAction` | Delays spawn slightly so the robot description publisher is already alive. |

## 5 Package Installation Updates

### 5.1 `CMakeLists.txt`

The package now installs all required runtime folders:

```cmake
install(
  DIRECTORY meshes rviz launch urdf
  DESTINATION share/${PROJECT_NAME}
)
```

This is important because Gazebo and RViz both use installed resources, not just source files.

### 5.2 `package.xml`

The package needs these runtime dependencies:

```xml
<exec_depend>xacro</exec_depend>
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
```

You still keep the display dependencies from earlier chapters:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `ros2launch`

Rebuild and source:

```bash
cd /github/racademy_ws
colcon build --packages-select racademy_description
source install/setup.bash
```

## 6 Running the Simulator

Launch Gazebo Sim with the robot:

```bash
ros2 launch racademy_description gazebo.launch.py
```

What should happen:

- Gazebo opens with an empty world
- `robot_state_publisher` publishes TF using the xacro-expanded robot
- `ros_gz_sim create` spawns `racademy`
- the `/clock` bridge starts

Useful checks:

```bash
ros2 topic echo /clock
ros2 topic echo /robot_description
```

You can also override launch arguments:

```bash
ros2 launch racademy_description gazebo.launch.py world:=empty.sdf entity_name:=racademy
```

## 7 Next Steps

- Add a proper drive plugin or ROS 2 control setup for wheel velocity commands.
- Add camera or IMU sensor plugins if you want simulated perception topics.
- Replace `empty.sdf` with a custom world once the robot model is stable.

— **End of Chapter 6**
