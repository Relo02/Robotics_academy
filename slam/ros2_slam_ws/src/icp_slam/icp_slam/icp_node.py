"""
icp_node.py — ICP scan-matching SLAM for a 2-D differential-drive robot
=========================================================================

Overview
--------
This node implements a minimal but complete online SLAM pipeline:

  1. **Scan subscription** — receives 2-D LiDAR scans from /scan.
  2. **ICP scan matching** — registers the current scan against the previous
     one to estimate the incremental rigid-body transformation (dx, dy, dtheta).
  3. **Pose accumulation** — integrates incremental transforms to maintain a
     global robot pose in the map frame.
  4. **Occupancy-grid mapping** — performs a simple ray-casting (Bresenham)
     to mark free/occupied cells in a 2-D OccupancyGrid.
  5. **Publishing** — continuously publishes the map and broadcasts the
     map → odom TF, which closes the loop with the EKF-filtered odom → base.

ICP Algorithm (vanilla point-to-point)
---------------------------------------
Given:
  P — source point cloud (current scan in robot frame)
  Q — target point cloud (previous scan in robot frame)

Iterate until convergence (or max_iter):
  (a) Correspondence: for each p_i in P, find nearest q_j in Q.
  (b) Compute optimal rotation R* and translation t* via SVD:
        μ_P = mean(P),  μ_Q = mean(Q)
        W   = Σ (p_i - μ_P)^T (q_i - μ_Q)
        U, S, Vt = SVD(W)
        R*  = V U^T            (handle reflection: det check)
        t*  = μ_Q - R* μ_P
  (c) Apply transform: P ← R* P + t*
  (d) Check convergence: ||t*|| < tol and angle(R*) < angle_tol.

The final cumulative transform is converted to (dx, dy, dtheta) and
integrated into the global pose estimate.

Equivalent terminal commands (without launch file)
--------------------------------------------------
  Start the node:
    ros2 run icp_slam icp_node \\
        --ros-args \\
        -p max_iter:=50 \\
        -p tolerance:=0.001 \\
        -p min_scan_dist:=0.5 \\
        -p map_resolution:=0.05 \\
        -p map_size:=200 \\
        -p use_sim_time:=true

  Inspect map:
    ros2 topic echo /map --no-arr   (header only)

  Inspect scan-match pose correction:
    ros2 topic echo /icp_pose

  View in RViz:
    ros2 run rviz2 rviz2
    — Add display: Map  → topic /map
    — Add display: Pose → topic /icp_pose
    — Add display: TF
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time as TimeMsg
import struct


# ======================================================================
# Pure-Python ICP (no external libraries required beyond numpy)
# ======================================================================

def scan_to_points(scan: LaserScan) -> np.ndarray:
    """
    Convert a LaserScan message to an (N, 2) array of Cartesian points.
    Discards points outside [range_min, range_max].
    """
    angles = np.linspace(scan.angle_min, scan.angle_max,
                         len(scan.ranges), endpoint=True)
    ranges = np.array(scan.ranges, dtype=np.float64)
    valid  = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack([r * np.cos(a), r * np.sin(a)])   # (N, 2)


def nearest_neighbour(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """
    Brute-force nearest-neighbour search.
    Returns indices into *target* for each point in *source*.
    For large point clouds, replace with scipy.spatial.cKDTree.query.

    Equivalent scipy call (faster):
      from scipy.spatial import cKDTree
      tree = cKDTree(target)
      _, idx = tree.query(source)
    """
    # Distance matrix  (N_src × N_tgt) — O(N²) but fine for 360-pt scans
    diff = source[:, None, :] - target[None, :, :]   # (N_src, N_tgt, 2)
    dist = np.linalg.norm(diff, axis=-1)              # (N_src, N_tgt)
    return np.argmin(dist, axis=1)                    # (N_src,)


def best_fit_transform(P: np.ndarray, Q: np.ndarray):
    """
    Compute the least-squares rigid-body transform that maps P → Q.

    Parameters
    ----------
    P : (N, 2)  source points
    Q : (N, 2)  corresponding target points (same ordering)

    Returns
    -------
    R : (2, 2)  rotation matrix
    t : (2,)    translation vector
    """
    mu_P = P.mean(axis=0)
    mu_Q = Q.mean(axis=0)

    # Centred clouds
    A = P - mu_P   # (N, 2)
    B = Q - mu_Q   # (N, 2)

    # Cross-covariance matrix  W = A^T B
    W = A.T @ B    # (2, 2)

    # SVD decomposition
    U, _S, Vt = np.linalg.svd(W)

    # Rotation (handle reflection to ensure det(R) = +1)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, d])
    R = Vt.T @ D @ U.T

    # Translation
    t = mu_Q - R @ mu_P

    return R, t


def icp(source: np.ndarray, target: np.ndarray,
        max_iter: int = 50, tol: float = 1e-3) -> tuple:
    """
    Point-to-point ICP.

    Returns
    -------
    R_total  : (2, 2)  cumulative rotation
    t_total  : (2,)    cumulative translation
    n_iter   : int     iterations until convergence
    """
    R_total = np.eye(2)
    t_total = np.zeros(2)
    P = source.copy()

    for i in range(max_iter):
        # (a) Find correspondences
        idx = nearest_neighbour(P, target)
        Q_corr = target[idx]

        # (b) Optimal transform for this iteration
        R, t = best_fit_transform(P, Q_corr)

        # (c) Apply to source cloud
        P = (R @ P.T).T + t

        # (d) Accumulate transform
        t_total = R @ t_total + t
        R_total = R @ R_total

        # (e) Convergence check
        angle = abs(math.atan2(R[1, 0], R[0, 0]))
        if np.linalg.norm(t) < tol and angle < tol:
            return R_total, t_total, i + 1

    return R_total, t_total, max_iter


# ======================================================================
# Occupancy grid helper (Bresenham ray casting)
# ======================================================================

def bresenham(x0: int, y0: int, x1: int, y1: int):
    """
    Bresenham's line algorithm — yields all integer cells on the line
    from (x0, y0) to (x1, y1).

    Used to mark *free* cells along each LiDAR ray before the hit cell.
    """
    dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
    dy = abs(y1 - y0); sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; x0 += sx
        if e2 <  dx:
            err += dx; y0 += sy


# ======================================================================
# ROS 2 node
# ======================================================================

class ICPSlamNode(Node):
    """Online SLAM via ICP scan matching + occupancy-grid mapping."""

    def __init__(self):
        super().__init__('icp_slam')

        # ---- parameters -----------------------------------------------
        self.declare_parameter('max_iter',      50)
        self.declare_parameter('tolerance',     0.001)
        self.declare_parameter('min_scan_dist', 0.5)    # min translation to trigger ICP [m]
        self.declare_parameter('map_resolution', 0.05)  # metres per cell
        self.declare_parameter('map_size',       200)   # cells per side (square map)
        self.declare_parameter('use_sim_time',   True)

        self._max_iter   = self.get_parameter('max_iter').value
        self._tol        = self.get_parameter('tolerance').value
        self._min_dist   = self.get_parameter('min_scan_dist').value
        self._res        = self.get_parameter('map_resolution').value
        size             = self.get_parameter('map_size').value

        # ---- map (log-odds grid) ---------------------------------------
        # log-odds: 0 = unknown, positive = occupied, negative = free
        self._map_size = size
        self._log_odds = np.zeros((size, size), dtype=np.float32)
        self._map_origin_x = -size * self._res / 2.0   # centre at (0,0)
        self._map_origin_y = -size * self._res / 2.0
        self._l_occ  =  0.85   # log-odds increment for occupied cell
        self._l_free = -0.40   # log-odds increment for free cell
        self._l_max  =  5.0    # saturation
        self._l_min  = -5.0

        # ---- state ----------------------------------------------------
        self._prev_scan   = None    # previous scan points (N, 2)
        self._pose        = np.zeros(3)    # [x, y, theta] in map frame
        self._prev_pose   = np.zeros(3)    # pose at last ICP trigger

        # ---- TF broadcaster -------------------------------------------
        self._tf_br = TransformBroadcaster(self)

        # ---- publishers -----------------------------------------------
        # ros2 topic echo /map
        self._map_pub  = self.create_publisher(OccupancyGrid, 'map',      10)
        # ros2 topic echo /icp_pose
        self._pose_pub = self.create_publisher(PoseStamped,   'icp_pose', 10)

        # ---- subscriber -----------------------------------------------
        # ros2 topic echo /scan
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)

        # Publish map at 1 Hz regardless of scan rate
        self.create_timer(1.0, self._publish_map)

        self.get_logger().info('ICP SLAM node started.')

    # ------------------------------------------------------------------
    # Scan callback
    # ------------------------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        """Called for every incoming LiDAR scan."""
        pts = scan_to_points(msg)
        if pts.shape[0] < 10:
            return   # not enough points for reliable ICP

        stamp = msg.header.stamp

        if self._prev_scan is None:
            # First scan: initialise map with it
            self._prev_scan = pts
            self._integrate_scan(pts, stamp)
            return

        # ---- ICP scan matching ----------------------------------------
        # Align current scan (source) to previous scan (target)
        R, t, n_iter = icp(pts, self._prev_scan,
                            max_iter=self._max_iter, tol=self._tol)

        # Extract incremental (dx, dy, dtheta) in the robot frame
        dx     = t[0]
        dy     = t[1]
        dtheta = math.atan2(R[1, 0], R[0, 0])

        # Rotate increment to the map frame and accumulate global pose
        cos_th = math.cos(self._pose[2])
        sin_th = math.sin(self._pose[2])
        self._pose[0] += cos_th * dx - sin_th * dy
        self._pose[1] += sin_th * dx + cos_th * dy
        self._pose[2] += dtheta
        self._pose[2]  = math.atan2(math.sin(self._pose[2]),
                                    math.cos(self._pose[2]))

        # ---- check minimum travel before updating reference scan ------
        moved = math.hypot(self._pose[0] - self._prev_pose[0],
                           self._pose[1] - self._prev_pose[1])
        if moved >= self._min_dist:
            self._integrate_scan(pts, stamp)
            self._prev_scan  = pts
            self._prev_pose  = self._pose.copy()

        # ---- broadcast map → odom TF -----------------------------------
        # This allows the rest of the TF chain (odom → base_footprint from EKF)
        # to be visualised correctly in RViz relative to the map frame.
        # Equivalent check: ros2 run tf2_tools view_frames
        self._broadcast_map_tf(stamp)

        # ---- publish pose ---------------------------------------------
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self._pose[0]
        pose_msg.pose.position.y = self._pose[1]
        th = self._pose[2]
        pose_msg.pose.orientation.z = math.sin(th / 2.0)
        pose_msg.pose.orientation.w = math.cos(th / 2.0)
        self._pose_pub.publish(pose_msg)

    # ------------------------------------------------------------------
    # Mapping: integrate a scan into the log-odds grid
    # ------------------------------------------------------------------

    def _integrate_scan(self, pts: np.ndarray, stamp):
        """Ray-cast each beam and update the log-odds occupancy grid."""
        ox, oy, oth = self._pose
        res  = self._res
        size = self._map_size

        # Robot cell in grid coordinates
        rx = int((ox - self._map_origin_x) / res)
        ry = int((oy - self._map_origin_y) / res)

        cos_th = math.cos(oth)
        sin_th = math.sin(oth)

        for p in pts:
            # Transform point from robot frame to map frame
            wx = ox + cos_th * p[0] - sin_th * p[1]
            wy = oy + sin_th * p[0] + cos_th * p[1]

            # Hit cell in grid coordinates
            hx = int((wx - self._map_origin_x) / res)
            hy = int((wy - self._map_origin_y) / res)

            # Bresenham ray: mark free cells
            for cx, cy in bresenham(rx, ry, hx, hy):
                if 0 <= cx < size and 0 <= cy < size:
                    if cx == hx and cy == hy:
                        # Hit cell → occupied
                        self._log_odds[cy, cx] = min(
                            self._log_odds[cy, cx] + self._l_occ, self._l_max)
                    else:
                        # Ray cell → free
                        self._log_odds[cy, cx] = max(
                            self._log_odds[cy, cx] + self._l_free, self._l_min)

    # ------------------------------------------------------------------
    # Map publisher (1 Hz timer)
    # ------------------------------------------------------------------

    def _publish_map(self):
        """Convert log-odds grid to ROS OccupancyGrid and publish."""
        grid = OccupancyGrid()
        grid.header.stamp    = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        grid.info.resolution = self._res
        grid.info.width      = self._map_size
        grid.info.height     = self._map_size
        grid.info.origin.position.x = self._map_origin_x
        grid.info.origin.position.y = self._map_origin_y
        grid.info.origin.orientation.w = 1.0

        # Convert log-odds to ROS probabilities:
        #   -1   = unknown (log-odds ≈ 0)
        #    0   = free
        #  100   = occupied
        lo = self._log_odds
        data = np.full(lo.shape, -1, dtype=np.int8)
        data[lo >  0.1]  = 100   # occupied
        data[lo < -0.1]  =   0   # free

        grid.data = data.flatten().tolist()
        self._map_pub.publish(grid)

    # ------------------------------------------------------------------
    # TF broadcast
    # ------------------------------------------------------------------

    def _broadcast_map_tf(self, stamp):
        """
        Broadcast the map → odom transform.

        The full TF chain is:
          map → odom  (from ICP SLAM, published here)
          odom → base_footprint  (from EKF node)
          base_footprint → base_link → lidar_link → ...  (from robot_state_publisher)

        Inspect with:
          ros2 run tf2_ros tf2_echo map base_footprint
        """
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id  = 'odom'

        # The ICP pose is map→base; we want map→odom.
        # For simplicity we treat odom ≈ map origin (zero offset here).
        # A production system would compute the difference between
        # the ICP pose and the EKF pose to get the map→odom correction.
        th = self._pose[2]
        tf_msg.transform.translation.x = self._pose[0]
        tf_msg.transform.translation.y = self._pose[1]
        tf_msg.transform.rotation.z    = math.sin(th / 2.0)
        tf_msg.transform.rotation.w    = math.cos(th / 2.0)
        self._tf_br.sendTransform(tf_msg)


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ICPSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
