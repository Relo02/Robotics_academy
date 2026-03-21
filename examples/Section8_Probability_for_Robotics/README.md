# Section 8 — Probability for Robotics

## Overview

This section introduces **probabilistic robotics** — the mathematical framework for reasoning under uncertainty. Real sensors and actuators are noisy. By modeling that noise statistically, we can represent not just a single pose estimate but a *probability distribution* over possible robot states. This is the foundation for all modern robot localization and mapping algorithms.

## Workspace Structure

```
bumperbot_ws/src/
├── bumperbot_msgs/             # Custom message/service definitions
├── bumperbot_description/      # Robot model
├── bumperbot_controller/       # Noisy controller (adds simulated sensor noise)
├── bumperbot_cpp_examples/     # C++ examples
└── bumperbot_py_examples/      # Python examples
```

## Key Implementation — `noisy_controller`

Located in `bumperbot_controller/src/noisy_controller.cpp`. This node extends the odometry controller from Section 7 by injecting **Gaussian noise** into the wheel encoder readings before integration.

### Noise Model

Each wheel's measured angular displacement $\hat{\phi}$ is modeled as:

$$\hat{\phi} = \phi_{\text{true}} + \epsilon, \qquad \epsilon \sim \mathcal{N}(0,\, \sigma^2)$$

where $\sigma$ is the standard deviation of the sensor noise.

### Effect on Pose Estimate

With noisy measurements, the integrated pose error grows over time. For a robot travelling a distance $d$ with per-step noise $\sigma$, the position uncertainty grows approximately as:

$$\sigma_{\text{pose}} \propto \sigma \sqrt{d}$$

This is the **random walk** property of integrated noise — the core motivation for filtering.

### Dual Odometry Output

The node publishes two parallel odometry streams so you can compare them:

| Topic | Frame | Description |
|-------|-------|-------------|
| `/bumperbot_controller/odom` | `odom → base_footprint` | Clean, noise-free estimate |
| `/bumperbot_controller/odom_noisy` | `odom → base_footprint_noisy` | Corrupted by Gaussian noise |

### Covariance Matrix

The odometry message covariance reflects the measurement uncertainty. For a diagonal noise model:

$$\Sigma = \begin{pmatrix} \sigma_x^2 & 0 & 0 \\ 0 & \sigma_y^2 & 0 \\ 0 & 0 & \sigma_\theta^2 \end{pmatrix}$$

A larger covariance signals lower confidence in the estimate.

## Running the Examples

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# Launch simulation
ros2 launch bumperbot_description gazebo.launch.py

# Launch the noisy controller
ros2 launch bumperbot_controller controller.launch.py

# Drive and compare clean vs noisy odometry
ros2 topic echo /bumperbot_controller/odom
ros2 topic echo /bumperbot_controller/odom_noisy

# Visualize both TF frames side by side in RViz
rviz2
```

## Key Concepts

| Concept | Description |
|---------|-------------|
| **Gaussian distribution** $\mathcal{N}(\mu, \sigma^2)$ | Most common noise model; defined by mean and variance |
| **Covariance matrix** | Encodes the uncertainty (and correlations) of a multi-dimensional estimate |
| **Random walk** | Noise accumulates with each integration step — drift grows as $\sqrt{t}$ |
| **Dead reckoning error** | Cumulative pose error from integrating noisy velocities |

## Dependencies

| Package | Purpose |
|---------|---------|
| `nav_msgs` | `Odometry` with covariance fields |
| `tf2_ros` | Broadcasting both clean and noisy TF frames |
| Standard C++ `<random>` | `std::normal_distribution` for Gaussian noise generation |

## What You Will Learn

- Why real-world odometry always drifts
- How to model sensor uncertainty with Gaussian distributions
- How covariance matrices represent multi-dimensional uncertainty
- How to inject and study the effect of measurement noise in simulation
- Why filtering (Section 9) is necessary to maintain accurate state estimates
