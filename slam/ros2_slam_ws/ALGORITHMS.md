# Algorithm Documentation: EKF Localization & ICP Scan Matching

This document provides step-by-step mathematical derivations for the two core algorithms implemented in this workspace.

---

## Table of Contents

1. [Extended Kalman Filter (EKF) — State Estimation](#1-extended-kalman-filter-ekf--state-estimation)
   - 1.1 [State Representation](#11-state-representation)
   - 1.2 [Motion Model](#12-motion-model)
   - 1.3 [Jacobian of the Motion Model](#13-jacobian-of-the-motion-model)
   - 1.4 [Prediction Step](#14-prediction-step)
   - 1.5 [Measurement Model](#15-measurement-model)
   - 1.6 [Update Step](#16-update-step)
   - 1.7 [Noise Matrices Q and R](#17-noise-matrices-q-and-r)
   - 1.8 [EKF Summary (pseudocode)](#18-ekf-summary-pseudocode)
2. [ICP Scan Matching](#2-icp-scan-matching)
   - 2.1 [Problem Statement](#21-problem-statement)
   - 2.2 [Scan Pre-processing](#22-scan-pre-processing)
   - 2.3 [Nearest-Neighbour Correspondence](#23-nearest-neighbour-correspondence)
   - 2.4 [Optimal Transform via SVD](#24-optimal-transform-via-svd)
   - 2.5 [Convergence & Iteration](#25-convergence--iteration)
   - 2.6 [Pose Integration](#26-pose-integration)
   - 2.7 [ICP Summary (pseudocode)](#27-icp-summary-pseudocode)
3. [Occupancy Grid Mapping](#3-occupancy-grid-mapping)
   - 3.1 [Log-Odds Representation](#31-log-odds-representation)
   - 3.2 [Inverse Sensor Model](#32-inverse-sensor-model)
   - 3.3 [Bresenham Ray Cast](#33-bresenham-ray-cast)
4. [TF Chain](#4-tf-chain)

---

## 1. Extended Kalman Filter (EKF) — State Estimation

**Source file:** `src/slam_ekf/slam_ekf/ekf_node.py`

The EKF is the nonlinear extension of the Kalman Filter. It linearises the motion and measurement functions around the current state estimate using first-order Taylor expansions (Jacobians).

### 1.1 State Representation

The robot state is a 5-dimensional vector:

$$\mathbf{x}_k = \begin{bmatrix} x \\ y \\ \theta \\ v \\ \omega \end{bmatrix}_k$$

where:
- $(x, y)$ — robot position in the `odom` frame $[\text{m}]$
- $\theta$ — heading (yaw) $[\text{rad}]$, wrapped to $[-\pi, \pi]$
- $v$ — longitudinal velocity $[\text{m/s}]$
- $\omega$ — angular velocity $[\text{rad/s}]$

The associated **state covariance** matrix $\mathbf{P}_k \in \mathbb{R}^{5 \times 5}$ represents the uncertainty in each dimension and the correlations between dimensions.

---

### 1.2 Motion Model

The robot obeys a **unicycle kinematics** model. Given control inputs $\mathbf{u}_k = [v_k,\, \omega_k]^\top$ (linear and angular velocities) and a time step $\Delta t$, the state transition is:

$$f(\mathbf{x}_{k-1}, \mathbf{u}_k) = \begin{bmatrix}
x + v \cos\theta \cdot \Delta t \\
y + v \sin\theta \cdot \Delta t \\
\theta + \omega \cdot \Delta t \\
v \\
\omega
\end{bmatrix}$$

> **Implementation note:** $v$ comes from the wheel encoder odometry (`/odom`), $\omega$ comes from the IMU gyroscope (`/imu/data`). Using the IMU for $\omega$ in the prediction step reduces heading drift compared to using wheel odometry alone.

The function $f$ is **nonlinear** in $\theta$ (because of $\cos\theta$ and $\sin\theta$). This is why we need the EKF instead of the standard (linear) Kalman Filter.

---

### 1.3 Jacobian of the Motion Model

To propagate the covariance, the EKF linearises $f$ around the current estimate by computing its Jacobian:

$$\mathbf{F}_k = \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\mathbf{x} = \hat{\mathbf{x}}_{k-1|k-1}}$$

The partial derivatives of each output with respect to each state variable:

| | $\partial / \partial x$ | $\partial / \partial y$ | $\partial / \partial \theta$ | $\partial / \partial v$ | $\partial / \partial \omega$ |
|---|---|---|---|---|---|
| $\partial x'$ | 1 | 0 | $-v\sin\theta\,\Delta t$ | $\cos\theta\,\Delta t$ | 0 |
| $\partial y'$ | 0 | 1 | $v\cos\theta\,\Delta t$  | $\sin\theta\,\Delta t$ | 0 |
| $\partial \theta'$ | 0 | 0 | 1 | 0 | $\Delta t$ |
| $\partial v'$ | 0 | 0 | 0 | 1 | 0 |
| $\partial \omega'$ | 0 | 0 | 0 | 0 | 1 |

In matrix form:

$$\mathbf{F}_k = \begin{bmatrix}
1 & 0 & -v\sin\theta\,\Delta t & \cos\theta\,\Delta t & 0 \\
0 & 1 &  v\cos\theta\,\Delta t & \sin\theta\,\Delta t & 0 \\
0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

---

### 1.4 Prediction Step

Using $\mathbf{F}_k$ and the process noise covariance $\mathbf{Q}_k$, the prediction step propagates both the state mean and the covariance forward in time:

$$\hat{\mathbf{x}}_{k|k-1} = f\!\left(\hat{\mathbf{x}}_{k-1|k-1},\, \mathbf{u}_k\right)$$

$$\mathbf{P}_{k|k-1} = \mathbf{F}_k\, \mathbf{P}_{k-1|k-1}\, \mathbf{F}_k^\top + \mathbf{Q}_k$$

The notation $\hat{\mathbf{x}}_{k|k-1}$ means "estimate of step $k$, given observations up to step $k-1$" (i.e., before the measurement update).

---

### 1.5 Measurement Model

The measurement $\mathbf{z}_k$ consists of the velocities extracted from the wheel odometry:

$$\mathbf{z}_k = \begin{bmatrix} v_{\text{odom}} \\ \omega_{\text{odom}} \end{bmatrix}$$

The measurement function $h(\mathbf{x})$ simply selects $v$ and $\omega$ from the state:

$$h(\mathbf{x}) = \begin{bmatrix} v \\ \omega \end{bmatrix} = \mathbf{H}\,\mathbf{x}$$

where the **measurement Jacobian** $\mathbf{H} \in \mathbb{R}^{2 \times 5}$ is:

$$\mathbf{H} = \begin{bmatrix}
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

Because $h$ is linear in $\mathbf{x}$, this step degenerates to the standard Kalman update (no Jacobian approximation needed here).

---

### 1.6 Update Step

Given a new measurement $\mathbf{z}_k$, the EKF corrects the predicted state:

**Innovation (measurement residual):**
$$\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1}) = \mathbf{z}_k - \mathbf{H}\,\hat{\mathbf{x}}_{k|k-1}$$

**Innovation covariance:**
$$\mathbf{S}_k = \mathbf{H}\,\mathbf{P}_{k|k-1}\,\mathbf{H}^\top + \mathbf{R}_k$$

**Kalman gain** (how much to trust the measurement vs. the prediction):
$$\mathbf{K}_k = \mathbf{P}_{k|k-1}\,\mathbf{H}^\top\,\mathbf{S}_k^{-1}$$

**State update:**
$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k\,\mathbf{y}_k$$

**Covariance update** (Joseph form used in implementation for numerical stability):
$$\mathbf{P}_{k|k} = \left(\mathbf{I} - \mathbf{K}_k\,\mathbf{H}\right)\mathbf{P}_{k|k-1}$$

> **Interpretation:** When $\mathbf{K}_k \to \mathbf{0}$ (high measurement noise $\mathbf{R}$), we trust the prediction. When $\mathbf{K}_k \to \mathbf{H}^{-1}$ (low $\mathbf{R}$, high $\mathbf{P}$), we trust the measurement.

---

### 1.7 Noise Matrices Q and R

**Process noise** $\mathbf{Q}_k$ — uncertainty introduced by the motion model per step:

$$\mathbf{Q} = \text{diag}(\sigma_x^2,\; \sigma_y^2,\; \sigma_\theta^2,\; \sigma_v^2,\; \sigma_\omega^2)$$

**Measurement noise** $\mathbf{R}_k$ — uncertainty in the wheel odometry readings:

$$\mathbf{R} = \text{diag}(\sigma_{v,\text{odom}}^2,\; \sigma_{\omega,\text{odom}}^2)$$

**Tuning guidelines:**

| Increase... | Effect |
|---|---|
| $\mathbf{Q}$ (process noise) | Filter reacts faster to measurements; prediction trusted less |
| $\mathbf{R}$ (measurement noise) | Filter relies more on prediction; smoother but slower to correct |

Default values (configurable via ROS parameters):
- $\sigma_{x} = \sigma_{y} = \sigma_\theta = 0.01$, $\sigma_v = \sigma_\omega = 0.05$
- $\sigma_{v,\text{odom}} = \sigma_{\omega,\text{odom}} = 0.1$

---

### 1.8 EKF Summary (pseudocode)

```
initialise: x = [0,0,0,0,0]^T,  P = 0.1 * I_5

on each /odom message at time t:
    dt = t - t_prev

    # --- Prediction ---
    v   = odom.twist.linear.x
    w   = imu_angular_velocity_z          # from last /imu/data callback
    x   = f(x, v, w, dt)                  # unicycle model
    F   = jacobian_F(v, x[2], dt)         # 5x5 analytical Jacobian
    P   = F @ P @ F.T + Q                 # covariance propagation

    # --- Update ---
    z   = [odom.twist.linear.x,
           odom.twist.angular.z]          # measurement vector
    y   = z - H @ x                       # innovation
    S   = H @ P @ H.T + R                 # innovation covariance
    K   = P @ H.T @ inv(S)                # Kalman gain
    x   = x + K @ y                       # state correction
    P   = (I - K @ H) @ P                 # covariance correction
    x[2] = wrap_to_pi(x[2])              # keep theta in [-pi, pi]

    publish x as /odometry/filtered
    broadcast TF: odom -> base_footprint
```

---

## 2. ICP Scan Matching

**Source file:** `src/icp_slam/icp_slam/icp_node.py`

### 2.1 Problem Statement

Given two 2-D point clouds:
- $\mathcal{P} = \{\mathbf{p}_i\}_{i=1}^{N}$ — **source** (current LiDAR scan in robot frame)
- $\mathcal{Q} = \{\mathbf{q}_j\}_{j=1}^{M}$ — **target** (previous LiDAR scan, same frame)

Find the rigid-body transformation $(\mathbf{R}^*, \mathbf{t}^*)$ that minimises the sum of squared distances between corresponding points:

$$(\mathbf{R}^*, \mathbf{t}^*) = \underset{\mathbf{R} \in SO(2),\; \mathbf{t} \in \mathbb{R}^2}{\arg\min}
\sum_{i=1}^{N} \left\| \mathbf{R}\,\mathbf{p}_i + \mathbf{t} - \mathbf{q}_{c(i)} \right\|^2$$

where $c(i)$ is the index of the nearest neighbour of $\mathbf{p}_i$ in $\mathcal{Q}$.

Because $c(i)$ depends on the current alignment of $\mathcal{P}$, this is a **chicken-and-egg** problem: we need correspondences to find the transform, and the transform to find correspondences. ICP solves it by alternating between these two steps until convergence.

---

### 2.2 Scan Pre-processing

Each `LaserScan` message is converted to Cartesian coordinates:

$$\mathbf{p}_i = \begin{bmatrix} r_i \cos\alpha_i \\ r_i \sin\alpha_i \end{bmatrix}$$

where $r_i$ is the range reading and $\alpha_i = \alpha_{\min} + i \cdot \Delta\alpha$ is the bearing angle. Points outside $[r_{\min}, r_{\max}]$ and non-finite values are discarded.

---

### 2.3 Nearest-Neighbour Correspondence

For each source point $\mathbf{p}_i$, find the closest target point:

$$c(i) = \underset{j}{\arg\min}\; \left\| \mathbf{p}_i - \mathbf{q}_j \right\|_2$$

**Implementation:** brute-force $O(NM)$ via vectorised distance matrix. For large point clouds, replace with a $k$-d tree ($O(N \log M)$):

```python
from scipy.spatial import cKDTree
tree = cKDTree(target)
_, idx = tree.query(source)
```

---

### 2.4 Optimal Transform via SVD

Given $N$ point pairs $\{(\mathbf{p}_i, \mathbf{q}_{c(i)})\}$, the least-squares rigid transform is computed analytically using SVD.

**Step 1 — Demean:**

$$\boldsymbol{\mu}_P = \frac{1}{N}\sum_i \mathbf{p}_i, \qquad
\boldsymbol{\mu}_Q = \frac{1}{N}\sum_i \mathbf{q}_{c(i)}$$

$$\tilde{\mathbf{p}}_i = \mathbf{p}_i - \boldsymbol{\mu}_P, \qquad
\tilde{\mathbf{q}}_i = \mathbf{q}_{c(i)} - \boldsymbol{\mu}_Q$$

**Step 2 — Cross-covariance matrix:**

$$\mathbf{W} = \sum_{i=1}^{N} \tilde{\mathbf{p}}_i\, \tilde{\mathbf{q}}_i^\top \;\in \mathbb{R}^{2 \times 2}$$

In matrix form: $\mathbf{W} = \tilde{\mathbf{P}}^\top \tilde{\mathbf{Q}}$ where rows are points.

**Step 3 — Singular Value Decomposition:**

$$\mathbf{W} = \mathbf{U}\, \mathbf{\Sigma}\, \mathbf{V}^\top$$

**Step 4 — Optimal rotation:**

$$\mathbf{R}^* = \mathbf{V}\, \mathbf{D}\, \mathbf{U}^\top$$

where $\mathbf{D} = \text{diag}(1,\; \det(\mathbf{V}\mathbf{U}^\top))$ corrects for reflections (ensures $\det(\mathbf{R}^*) = +1$, i.e. $\mathbf{R}^* \in SO(2)$).

> **Why the reflection check?** SVD minimises $\|\mathbf{R}\mathbf{P} - \mathbf{Q}\|_F$ but allows improper rotations (reflections, $\det = -1$). Multiplying by $\mathbf{D}$ constrains the solution to the rotation group $SO(2)$.

**Step 5 — Optimal translation:**

$$\mathbf{t}^* = \boldsymbol{\mu}_Q - \mathbf{R}^*\, \boldsymbol{\mu}_P$$

This is the unique translation that aligns the centroids after the optimal rotation is applied.

**Proof of optimality (sketch):**

The objective $\sum_i \|\mathbf{R}\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i\|^2$ is convex in $\mathbf{t}$ for fixed $\mathbf{R}$. Setting $\partial/\partial\mathbf{t} = 0$ gives $\mathbf{t} = \boldsymbol{\mu}_Q - \mathbf{R}\boldsymbol{\mu}_P$. Substituting back and expanding, the residual reduces to $-2\,\text{tr}(\mathbf{\Sigma}\mathbf{D})$, which is maximised (objective minimised) when $\mathbf{D}$ is as above and the SVD is used for $\mathbf{R}$.

---

### 2.5 Convergence & Iteration

After computing $(\mathbf{R}^*, \mathbf{t}^*)$ for iteration $k$:

1. Apply the transform to the source cloud: $\mathbf{p}_i \leftarrow \mathbf{R}^*\mathbf{p}_i + \mathbf{t}^*$
2. Accumulate the total transform:

$$\mathbf{t}_{\text{total}} \leftarrow \mathbf{R}^*\, \mathbf{t}_{\text{total}} + \mathbf{t}^*$$
$$\mathbf{R}_{\text{total}} \leftarrow \mathbf{R}^*\, \mathbf{R}_{\text{total}}$$

3. Check convergence:

$$\|\mathbf{t}^*\|_2 < \epsilon_t \quad \text{and} \quad |\text{atan2}(R^*_{10}, R^*_{00})| < \epsilon_\theta$$

with $\epsilon_t = \epsilon_\theta = 0.001$ (default). If both conditions hold, terminate early.

**Maximum iterations:** 50 (default). ICP is not guaranteed to find the global minimum; the initial alignment matters. For a differential-drive robot at 10 Hz LiDAR, successive scans overlap heavily, so convergence is reliable.

---

### 2.6 Pose Integration

The converged incremental transform $(\mathbf{R}_{\text{total}}, \mathbf{t}_{\text{total}})$ represents the robot motion between the two scans **in the robot frame at time $k-1$**.

To integrate it into the global pose $[x, y, \theta]^\top$ (in the map frame):

$$\Delta x_r = t_{\text{total},0}, \quad
\Delta y_r = t_{\text{total},1}, \quad
\Delta\theta = \text{atan2}(R_{\text{total},10},\, R_{\text{total},00})$$

Rotate the displacement from robot frame to map frame:

$$x \leftarrow x + \cos\theta\, \Delta x_r - \sin\theta\, \Delta y_r$$
$$y \leftarrow y + \sin\theta\, \Delta x_r + \cos\theta\, \Delta y_r$$
$$\theta \leftarrow \text{wrap}(\theta + \Delta\theta)$$

The reference scan is only updated when the robot has moved at least `min_scan_dist` (default 0.5 m) to avoid accumulating small-angle ICP errors at standstill.

---

### 2.7 ICP Summary (pseudocode)

```
function icp(P, Q, max_iter=50, tol=0.001):
    R_total = I_2
    t_total = [0, 0]

    for k in 1..max_iter:
        # Correspondence
        for each p_i in P:
            c(i) = argmin_j ||p_i - q_j||

        Q_corr = Q[c]    # (N, 2) matched target points

        # Optimal transform
        mu_P = mean(P);  mu_Q = mean(Q_corr)
        W    = (P - mu_P).T @ (Q_corr - mu_Q)    # cross-covariance
        U, S, Vt = SVD(W)
        d    = det(Vt.T @ U.T)
        R*   = Vt.T @ diag(1, d) @ U.T
        t*   = mu_Q - R* @ mu_P

        # Apply and accumulate
        P        = (R* @ P.T).T + t*
        t_total  = R* @ t_total + t*
        R_total  = R* @ R_total

        # Convergence check
        if ||t*|| < tol and |angle(R*)| < tol:
            break

    return R_total, t_total
```

---

## 3. Occupancy Grid Mapping

**Source file:** `src/icp_slam/icp_slam/icp_node.py` — `_integrate_scan()`

### 3.1 Log-Odds Representation

Each cell $(i, j)$ of the map stores a **log-odds** value $l_{ij}$:

$$l_{ij} = \log\frac{P(m_{ij} = 1 \mid \mathbf{z}_{1:t}, \mathbf{x}_{1:t})}{P(m_{ij} = 0 \mid \mathbf{z}_{1:t}, \mathbf{x}_{1:t})}$$

Log-odds are additive across time, which makes the update rule a simple increment/decrement — no multiplication of probabilities needed.

**Inverse log-odds (to recover probability):**
$$P(m_{ij} = 1) = 1 - \frac{1}{1 + e^{l_{ij}}}$$

### 3.2 Inverse Sensor Model

For a LiDAR ray that terminates at cell $(h_x, h_y)$ (hit), the update rules are:

| Cell type | Update |
|---|---|
| **Hit cell** (end of ray) | $l_{ij} \leftarrow \text{clip}(l_{ij} + l_{\text{occ}},\; l_{\min},\; l_{\max})$ |
| **Free cell** (along ray) | $l_{ij} \leftarrow \text{clip}(l_{ij} + l_{\text{free}},\; l_{\min},\; l_{\max})$ |

Default values: $l_{\text{occ}} = 0.85$, $l_{\text{free}} = -0.40$, $l_{\max} = 5$, $l_{\min} = -5$.

The asymmetry $|l_{\text{occ}}| > |l_{\text{free}}|$ reflects that a hit cell is more informative (a reflection is detected) than a free cell (just "no reflection along the beam").

### 3.3 Bresenham Ray Cast

To enumerate the integer grid cells along a ray from the robot cell $(r_x, r_y)$ to the hit cell $(h_x, h_y)$, the Bresenham line algorithm is used:

```
dx = |hx - rx|;  sx = sign(hx - rx)
dy = |hy - ry|;  sy = sign(hy - ry)
err = dx - dy
while (x, y) != (hx, hy):
    yield (x, y)
    e2 = 2 * err
    if e2 > -dy:  err -= dy;  x += sx
    if e2 <  dx:  err += dx;  y += sy
yield (hx, hy)   # hit cell
```

This runs in $O(L)$ where $L = \max(|dx|, |dy|)$ is the ray length in cells.

The map is published as a `nav_msgs/OccupancyGrid` with values:
- `-1` — unknown ($|l_{ij}| \leq 0.1$)
- `0` — free ($l_{ij} < -0.1$)
- `100` — occupied ($l_{ij} > 0.1$)

---

## 4. TF Chain

The complete transform tree when both EKF and ICP nodes are running:

```
map
 └── odom                (ICP node: map→odom broadcast)
      └── base_footprint (EKF node: odom→base_footprint broadcast)
           └── base_link (robot_state_publisher: fixed joint from URDF)
                ├── left_wheel  (continuous joint, updated from /joint_states)
                ├── right_wheel (continuous joint, updated from /joint_states)
                ├── lidar_link  (fixed joint → /scan arrives in this frame)
                ├── imu_link    (fixed joint → /imu/data arrives in this frame)
                └── stereo_camera_link
                     ├── camera_left_link   → camera_left_optical_link
                     └── camera_right_link  → camera_right_optical_link
```

**TF monitoring commands:**

```bash
# Visualise the full tree as a PDF
ros2 run tf2_tools view_frames

# Query a specific transform
ros2 run tf2_ros tf2_echo map base_footprint

# Check if TF is being published
ros2 topic echo /tf --no-arr
ros2 topic echo /tf_static --no-arr
```

**Role of each transform:**

| Transform | Publisher | Purpose |
|---|---|---|
| `map → odom` | `icp_slam` node | Corrects odometry drift using scan-match result |
| `odom → base_footprint` | `slam_ekf` node | Filtered localisation within the odom frame |
| `base_footprint → base_link` | `robot_state_publisher` | Fixed height offset (wheel radius) |
| `base_link → *_link` | `robot_state_publisher` | All fixed and moving sensor/joint frames from URDF |
