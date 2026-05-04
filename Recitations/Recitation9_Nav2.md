# Chapter 9 - Nav2 Navigation on `racademy_ws`

> _Goal_: present an autonomous navigation example (global planning + local control + behavior tree execution) on the `racademy` robot using the prepared `racademy_nav2` package.

> _Teaching note_: this chapter presents an already-prepared example. Students are not required to create packages or modify files.

---

## 1 Why a Separate Nav2 Package?

Keeping Nav2 in its own package gives clean boundaries:

1. `racademy_description`: robot model, URDF, Gazebo model assets.
2. `racademy_controller`: low-level actuation and odometry.
3. `racademy_slam`: mapping/localization stack and LiDAR bridge.
4. `racademy_nav2`: navigation tuning and launch orchestration.

This avoids mixing mission-level navigation parameters with robot description assets.

---

## 2 Nav2 Pipeline in This Course

For this recitation, the runtime chain is:

1. `slam_toolbox` localization publishes `map -> odom` and `/map`.
2. Nav2 global costmap consumes `/map` + `/scan`.
3. Nav2 planner computes a global path.
4. Nav2 controller computes local velocity commands.
5. `cmd_vel_relay` forwards `/cmd_vel` to `/diff_drive_controller/cmd_vel_unstamped`.
6. `diff_drive_controller` actuates wheels and publishes `/diff_drive_controller/odom`.

---

## 3 Prepared `racademy_nav2` Package Contents

Already available in:

- `racademy_ws/src/racademy_nav2/`

Reference files used in this example:

1. `launch/nav2_with_slam.launch.py`
2. `config/nav2_params.yaml`
3. `scripts/cmd_vel_relay.py`
4. `package.xml`
5. `CMakeLists.txt`

---

## 4 Launch Architecture

`nav2_with_slam.launch.py` brings up:

1. `racademy_slam/launch/slam_localization.launch.py`
2. `nav2_bringup/launch/navigation_launch.py`
3. `racademy_nav2/cmd_vel_relay.py`

Important launch arguments:

- `use_sim_time` (default `true`)
- `world` (default `empty.sdf`)
- `map_file_name` (default `/home/ros2user/ros2_ws/maps/racademy_graph`)
- `slam_params_file` (defaults to `racademy_slam` localization config)
- `nav2_params_file` (defaults to `racademy_nav2/config/nav2_params.yaml`)

---

## 5 Nav2 Parameter Highlights

The `nav2_params.yaml` is tuned for your diff-drive setup:

1. `controller_server.odom_topic: /diff_drive_controller/odom`
2. `velocity_smoother.enable_stamped_cmd_vel: false`
3. `behavior_server.enable_stamped_cmd_vel: false`
4. Local/global costmaps subscribe to `/scan`
5. Global costmap static layer subscribes to `/map`
6. Controller plugin set to regulated pure pursuit

This aligns Nav2 with the SLAM + controller interfaces from Chapter 8.

---

## 6 Demo Execution

### 6.1 Optional runtime start

```bash
cd ~/ros2_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 6.2 Start the prepared Nav2 + localization stack

```bash
ros2 launch racademy_nav2 nav2_with_slam.launch.py
```

If your serialized graph is elsewhere:

```bash
ros2 launch racademy_nav2 nav2_with_slam.launch.py map_file_name:=/home/ros2user/ros2_ws/maps/racademy_graph
```

---

## 7 Validate Interfaces Before Sending Goals

Check core topics and TF:

```bash
ros2 topic list | grep -E '^/map|^/scan|^/diff_drive_controller/odom|^/cmd_vel'
ros2 topic echo /map --once
ros2 topic echo /scan --once
ros2 topic echo /diff_drive_controller/odom --once
ros2 run tf2_ros tf2_echo map base_link
```

Check relay output:

```bash
ros2 topic hz /cmd_vel
ros2 topic hz /diff_drive_controller/cmd_vel_unstamped
```

---

## 8 Demonstrate Navigation Goals

For the live demo in RViz2:

1. Start RViz2.
2. Set `Fixed Frame` to `map`.
3. Add displays: `Map`, `LaserScan`, `TF`, `Global Costmap`, `Local Costmap`.
4. Use `2D Nav Goal` to send a goal.

CLI alternative for demonstration:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 9 Common Failure Modes

1. No movement, but Nav2 active:
- check `/diff_drive_controller/cmd_vel_unstamped` is receiving commands.
- verify `cmd_vel_relay` node is running.

2. Planner fails to create path:
- check `/map` is being published from SLAM localization.
- verify global costmap static layer is receiving map.

3. Robot footprint collides too aggressively:
- tune `robot_radius`, inflation radius, and obstacle layer ranges.

4. Localization drifts:
- verify `map_file_name` points to the correct serialized pose-graph base path.

---

## 10 Exercises

1. Reduce `desired_linear_vel` and compare path tracking smoothness.
2. Increase local costmap update rate and evaluate controller responsiveness.
3. Introduce moving obstacles and tune inflation radius for safer clearance.
4. Compare navigation performance with different SLAM localization buffer sizes.

---

## 11 Summary

You now have a dedicated `racademy_nav2` package that integrates with your existing `racademy_slam` workflow and provides a reproducible navigation bringup for the `racademy` robot.
