# SLAM Notebooks & ROS 2 Implementation

This folder contains two complementary learning tracks for probabilistic robotics and SLAM:

1. **Jupyter Notebooks** — theory-first, self-contained implementations in pure Python.
2. **ROS 2 Workspace** (`ros2_slam_ws/`) — the same algorithms running on a simulated differential-drive robot in Gazebo + RViz.

---

## Part 1 — Notebooks

The notebooks are organised in four modules with a pedagogical progression.

```
notebooks/
├── 0_introduction/
├── 1_kalman_filters/
├── 2_particle_filters/
├── 3_graph_based/
├── figures/             ← shared assets used by all notebooks
```

Figures are referenced with relative paths (`../figures/…`) and resolve automatically from any notebook subdirectory.

---

### Module 0 — Introduction

| Notebook | Description |
|---|---|
| `0_introduction/0_intro.ipynb` | Course overview: motivation, notation, the SLAM problem statement, and a map of the modules ahead. |

---

### Module 1 — Kalman Filters

| Notebook | Key concepts |
|---|---|
| `1_kalman_filters/1_bayes.ipynb` | Bayes filter, recursive state estimation, belief distributions. |
| `1_kalman_filters/2_models.ipynb` | Motion models (velocity, odometry) and sensor models (range-bearing). Gaussian noise, linearisation. |
| `1_kalman_filters/3_kalman_filters.ipynb` | Linear Kalman Filter derivation: prediction step ($\mathbf{P} = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$), update step (Kalman gain $\mathbf{K}$), state correction. |
| `1_kalman_filters/4_ekf_slam.ipynb` | Extended Kalman Filter for simultaneous localisation and mapping: joint state vector $\mathbf{x} = [x, y, \theta, m_1, \ldots, m_N]^\top$, Jacobian linearisation, landmark data association. |

**Core equations (EKF):**

Prediction:
$$\hat{\mathbf{x}}_{k|k-1} = f(\mathbf{x}_{k-1|k-1}, \mathbf{u}_k)$$
$$\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^\top + \mathbf{Q}_k$$

Update:
$$\mathbf{K}_k = \mathbf{P}_{k|k-1}\mathbf{H}_k^\top \left(\mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^\top + \mathbf{R}_k\right)^{-1}$$
$$\mathbf{x}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \left(\mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1})\right)$$
$$\mathbf{P}_{k|k} = \left(\mathbf{I} - \mathbf{K}_k \mathbf{H}_k\right)\mathbf{P}_{k|k-1}$$

---

### Module 2 — Particle Filters

| Notebook | Key concepts |
|---|---|
| `2_particle_filters/1_grid_maps.ipynb` | Occupancy grid maps, log-odds update rule, ray casting. |
| `2_particle_filters/2_particle_filter.ipynb` | Monte-Carlo Localisation (MCL): sampling, importance weights, resampling (systematic, stratified). |
| `2_particle_filters/3_fast_slam.ipynb` | FastSLAM 1.0/2.0: per-particle EKF for each landmark, Rao-Blackwellised factorisation. |
| `2_particle_filters/4_grid_based_slam.ipynb` | Grid-based FastSLAM (particle filter over poses, shared grid map), scan-matching correction. |

**Log-odds update:**

$$l_{t,i} = l_{t-1,i} + \text{inverse\_sensor\_model}(\mathbf{m}_i, \mathbf{x}_t, \mathbf{z}_t) - l_0$$

---

### Module 3 — Graph-Based SLAM

| Notebook | Key concepts |
|---|---|
| `3_graph_based/1_least_squares.ipynb` | Least-squares estimation, Gauss-Newton, sparse linear systems $(\mathbf{H}\boldsymbol{\xi} = \mathbf{b})$. |
| `3_graph_based/2_least_squares_slam.ipynb` | Pose-graph SLAM: nodes are robot poses, edges are odometry/loop-closure constraints; solved via $\mathbf{H}^{-1}\mathbf{b}$. |
| `3_graph_based/3_landmark_graph_slam.ipynb` | Landmark graph SLAM: heterogeneous graph with both pose nodes and landmark nodes; marginalisation. |

**Pose-graph normal equations:**

$$\mathbf{H} = \sum_{\langle i,j \rangle} \mathbf{J}_{ij}^\top \mathbf{\Omega}_{ij} \mathbf{J}_{ij}, \qquad
\mathbf{b} = \sum_{\langle i,j \rangle} \mathbf{J}_{ij}^\top \mathbf{\Omega}_{ij} \mathbf{e}_{ij}$$
$$\Delta\boldsymbol{\xi} = -\mathbf{H}^{-1}\mathbf{b}$$

---

## Part 2 — ROS 2 Workspace (`ros2_slam_ws/`)

The workspace implements the core notebook algorithms inside ROS 2, running on a differential-drive robot with **2-D LiDAR**, **stereo camera**, and **IMU**, simulated in Gazebo and visualised in RViz.

```
ros2_slam_ws/
└── src/
    ├── diff_drive_description/   ← robot URDF, Gazebo plugins, ros2_control
    ├── slam_ekf/                 ← EKF localization node (Module 1)
    ├── icp_slam/                 ← ICP scan-matching + mapping node (Module 2–3)
    └── slam_bringup/             ← master launch, RViz config
```

---

### Robot description (`diff_drive_description`)

A Xacro/URDF model of a ground robot with:

| Sensor | Frame | ROS topic |
|---|---|---|
| 2-D LiDAR (360°, 12 m range) | `lidar_link` | `/scan` |
| Stereo camera (12 cm baseline, 640×480 @ 30 Hz) | `camera_left_optical_link` / `camera_right_optical_link` | `/stereo/left/image_raw`, `/stereo/right/image_raw` |
| IMU (100 Hz) | `imu_link` | `/imu/data` |

Actuation is handled by **ros2_control**:
- `joint_state_broadcaster` — publishes `/joint_states`.
- `diff_drive_controller` — subscribes to `/cmd_vel`, publishes `/odom`.

Spawn the robot in Gazebo:
```bash
ros2 launch diff_drive_description spawn.launch.py
```

---

### EKF state estimation (`slam_ekf`) — *Module 1 in ROS 2*

Implements the **Extended Kalman Filter** from notebook `1_kalman_filters/3_kalman_filters.ipynb` and `4_ekf_slam.ipynb`.

**State vector:** $\mathbf{x} = [x,\; y,\; \theta,\; v,\; \omega]^\top$

**Inputs:**
- `/odom` (`nav_msgs/Odometry`) — wheel encoder velocities.
- `/imu/data` (`sensor_msgs/Imu`) — angular velocity used in the prediction step.

**Outputs:**
- `/odometry/filtered` (`nav_msgs/Odometry`) — posterior pose with covariance.
- TF broadcast: `odom → base_footprint`.

**Motion model** (unicycle):

$$x' = x + v \cos\theta \cdot \Delta t, \quad
y' = y + v \sin\theta \cdot \Delta t, \quad
\theta' = \theta + \omega \cdot \Delta t$$

**Jacobian** $\mathbf{F} = \partial f / \partial \mathbf{x}$:

$$\mathbf{F} = \begin{bmatrix}
1 & 0 & -v\sin\theta\,\Delta t & \cos\theta\,\Delta t & 0 \\
0 & 1 &  v\cos\theta\,\Delta t & \sin\theta\,\Delta t & 0 \\
0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

Run standalone:
```bash
ros2 run slam_ekf ekf_node \
    --ros-args \
    -p process_noise_v:=0.05 \
    -p measurement_noise_v:=0.1 \
    -p use_sim_time:=true
```

---

### ICP SLAM (`icp_slam`) — *Modules 2 & 3 in ROS 2*

Implements **Iterative Closest Point** scan matching (notebook concept from Module 3) combined with **log-odds occupancy mapping** (Module 2, `2_particle_filters/1_grid_maps.ipynb`).

**Input:** `/scan` (`sensor_msgs/LaserScan`).

**Outputs:**
- `/map` (`nav_msgs/OccupancyGrid`) — live 2-D occupancy grid.
- `/icp_pose` (`geometry_msgs/PoseStamped`) — ICP-derived global pose.
- TF broadcast: `map → odom`.

**ICP core (SVD solution):**

$$\mathbf{W} = \sum_i (\mathbf{p}_i - \boldsymbol{\mu}_P)^\top (\mathbf{q}_i - \boldsymbol{\mu}_Q)$$
$$\mathbf{U}, \mathbf{S}, \mathbf{V}^\top = \text{SVD}(\mathbf{W})$$
$$\mathbf{R}^* = \mathbf{V}\,\text{diag}(1,\,\det(\mathbf{V}\mathbf{U}^\top))\,\mathbf{U}^\top, \qquad
\mathbf{t}^* = \boldsymbol{\mu}_Q - \mathbf{R}^*\boldsymbol{\mu}_P$$

**TF chain completed by ICP + EKF:**

```
map ──(ICP)──► odom ──(EKF)──► base_footprint ──(RSP)──► base_link ──► lidar_link
                                                                   └──► camera_left_link
                                                                   └──► imu_link
```

Run standalone:
```bash
ros2 run icp_slam icp_node \
    --ros-args \
    -p max_iter:=50 \
    -p map_resolution:=0.05 \
    -p map_size:=200 \
    -p use_sim_time:=true
```

---

### Full bringup (`slam_bringup`)

Launches every component in the correct order with a single command:

```bash
ros2 launch slam_bringup slam_full.launch.py
```

Once running, drive the robot:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2}, angular: {z: 0.3}}" -r 10
```

Inspect topics:
```bash
ros2 topic list
ros2 topic echo /odometry/filtered
ros2 topic echo /map --no-arr
ros2 run tf2_tools view_frames
```

---

### Build the workspace

```bash
cd ros2_slam_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Dependencies: ROS 2 Humble (or later), `gazebo_ros_pkgs`, `ros2_control`, `diff_drive_controller`, `robot_state_publisher`, `rviz2`, `xacro`, `numpy`.

---

## Notebook ↔ ROS 2 mapping

| Notebook | ROS 2 equivalent |
|---|---|
| `1_kalman_filters/3_kalman_filters.ipynb` | `slam_ekf/slam_ekf/ekf_node.py` — `_predict()`, `_update()` |
| `1_kalman_filters/4_ekf_slam.ipynb` | `slam_ekf` — joint state estimation with odom + IMU fusion |
| `2_particle_filters/1_grid_maps.ipynb` | `icp_slam/icp_slam/icp_node.py` — `_integrate_scan()`, Bresenham ray cast |
| `3_graph_based/1_least_squares.ipynb` | `icp_slam` — SVD-based optimal transform in `best_fit_transform()` |
| `3_graph_based/2_least_squares_slam.ipynb` | `icp_slam` — incremental pose-graph integration via ICP |
| URDF / ros2_control | `diff_drive_description` — robot model, sensors, controllers |
