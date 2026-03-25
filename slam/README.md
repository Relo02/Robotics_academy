# SLAM Notebooks

This folder contains a theory-first, self-contained learning track for probabilistic robotics and SLAM implemented in pure Python Jupyter notebooks.

---

## Notebooks

### How to use the notebooks

Each notebook is **self-contained**: it imports only the standard scientific Python stack (`numpy`, `scipy`, `matplotlib`) and defines every helper function inline. No ROS 2 installation is required to run them.

**Prerequisites**

```bash
pip install -r slam/requirements.txt   # numpy, scipy, matplotlib, jupyterlab
```

**Launch JupyterLab**

```bash
cd slam
jupyter lab
```

Open notebooks in module order (0 → 1 → 2 → 3). Each module assumes familiarity with the concepts introduced in the previous one.

**Navigation tips**

- Run all cells top-to-bottom with **Run → Run All Cells** (or `Shift+Enter` cell by cell).
- Figures are loaded from `../figures/`; keep the working directory inside the module subfolder.
- Interactive widgets (sliders, animations) require `ipywidgets` — already included in `requirements.txt`.
- Each notebook ends with a short **exercises** section; completing them is the fastest way to consolidate the theory.

---

### Notebooks overview

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
| `0_introduction/0_intro.ipynb` | Presents motivation, notation, the SLAM problem statement, and a map of the modules ahead. |

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

