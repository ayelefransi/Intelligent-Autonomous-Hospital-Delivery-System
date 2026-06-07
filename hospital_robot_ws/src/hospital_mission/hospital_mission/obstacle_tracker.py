#!/usr/bin/env python3
"""
Dynamic Obstacle Tracker
=========================
Processes LiDAR scans to detect, cluster, track and classify
dynamic obstacles in the hospital environment.

Pipeline:
  1. LiDAR scan -> point cloud conversion
  2. DBSCAN clustering to segment obstacles
  3. Kalman filter tracking per cluster
  4. Velocity estimation and trajectory prediction
  5. Classification: predictable vs unpredictable
  6. Publish tracked obstacles for Nav2 costmap

ROS 2 Jazzy
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String

import json
import math
import time
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from enum import Enum


# ===================== OBSTACLE CLASSES ===================

class ObstacleType(Enum):
    UNKNOWN       = 0
    PERSON        = 1   # Doctors, nurses, patients
    CART          = 2   # Medical carts, wheelchairs
    STATIC        = 3   # Furniture, walls (false positive)
    ROBOT         = 4   # Other delivery robots


class ObstacleBehavior(Enum):
    UNPREDICTABLE = 0   # Doctors, emergency staff
    SEMI_PREDICTABLE = 1  # Nurses with carts on known routes
    PREDICTABLE   = 2   # Cleaning robots, scheduled movement


# ===================== KALMAN FILTER ======================

class KalmanFilter2D:
    """
    2D Kalman filter for obstacle tracking.
    State: [x, y, vx, vy]
    Measurement: [x, y]
    """

    def __init__(self, x: float, y: float):
        # State: [x, y, vx, vy]
        self.x = np.array([x, y, 0.0, 0.0], dtype=np.float64)

        # State covariance
        self.P = np.eye(4) * 1.0

        # State transition matrix
        self.F = np.eye(4)  # Updated with dt each step

        # Measurement matrix (we observe x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float64)

        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.5, 0.5])

        # Measurement noise
        self.R = np.diag([0.05, 0.05])

    def predict(self, dt: float):
        """Predict next state."""
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray):
        """Update with measurement [x, y]."""
        y  = z - self.H @ self.x
        S  = self.H @ self.P @ self.H.T + self.R
        K  = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def position(self) -> tuple[float, float]:
        return (float(self.x[0]), float(self.x[1]))

    @property
    def velocity(self) -> tuple[float, float]:
        return (float(self.x[2]), float(self.x[3]))

    @property
    def speed(self) -> float:
        vx, vy = self.velocity
        return math.sqrt(vx * vx + vy * vy)

    def predict_position(self, t_ahead: float) -> tuple[float, float]:
        """Predict position t_ahead seconds into the future."""
        px = self.x[0] + self.x[2] * t_ahead
        py = self.x[1] + self.x[3] * t_ahead
        return (float(px), float(py))


# ===================== TRACKED OBSTACLE ===================

@dataclass
class TrackedObstacle:
    obstacle_id:  int
    kf:           KalmanFilter2D
    obs_type:     ObstacleType    = ObstacleType.UNKNOWN
    behavior:     ObstacleBehavior = ObstacleBehavior.UNPREDICTABLE
    radius:       float           = 0.3
    hits:         int             = 0    # Consecutive detections
    misses:       int             = 0    # Consecutive missed detections
    first_seen:   float           = field(default_factory=time.time)
    last_seen:    float           = field(default_factory=time.time)

    @property
    def confirmed(self) -> bool:
        """Obstacle confirmed after 3+ consecutive detections."""
        return self.hits >= 3

    @property
    def stale(self) -> bool:
        """Obstacle not seen for 2 seconds."""
        return self.misses >= 4

    @property
    def age_seconds(self) -> float:
        return time.time() - self.first_seen


# ===================== DBSCAN CLUSTERING ==================

def dbscan_cluster(points: np.ndarray,
                   eps: float = 0.3,
                   min_pts: int = 3) -> list[np.ndarray]:
    """
    Simple DBSCAN implementation for LiDAR point clustering.
    Returns list of point arrays (one per cluster).

    Uses distance-based region growing without scipy dependency.
    """
    if len(points) == 0:
        return []

    n = len(points)
    labels = np.full(n, -1, dtype=int)  # -1 = unvisited
    cluster_id = 0

    def region_query(idx: int) -> list[int]:
        neighbors = []
        for j in range(n):
            if j != idx:
                dx = points[idx, 0] - points[j, 0]
                dy = points[idx, 1] - points[j, 1]
                if dx * dx + dy * dy <= eps * eps:
                    neighbors.append(j)
        return neighbors

    for i in range(n):
        if labels[i] != -1:
            continue

        neighbors = region_query(i)

        if len(neighbors) < min_pts:
            labels[i] = -2  # Noise
            continue

        labels[i] = cluster_id
        seed_set = list(neighbors)

        j = 0
        while j < len(seed_set):
            q = seed_set[j]
            if labels[q] == -2:
                labels[q] = cluster_id
            if labels[q] == -1:
                labels[q] = cluster_id
                q_neighbors = region_query(q)
                if len(q_neighbors) >= min_pts:
                    seed_set.extend(q_neighbors)
            j += 1

        cluster_id += 1

    clusters = []
    for cid in range(cluster_id):
        mask = labels == cid
        if mask.sum() >= min_pts:
            clusters.append(points[mask])

    return clusters


def classify_obstacle(cluster: np.ndarray,
                       speed: float,
                       radius: float) -> tuple[ObstacleType, ObstacleBehavior]:
    """
    Classify obstacle by size and speed.

    Size thresholds (approximate):
      person:  radius 0.2-0.5m
      cart:    radius 0.3-0.8m
      static:  radius > 0.5m, speed < 0.05 m/s
    """
    if speed < 0.05:
        if radius > 0.5:
            return ObstacleType.STATIC, ObstacleBehavior.PREDICTABLE
        return ObstacleType.UNKNOWN, ObstacleBehavior.UNPREDICTABLE

    if radius < 0.5:
        if speed > 0.8:
            return ObstacleType.PERSON, ObstacleBehavior.UNPREDICTABLE
        return ObstacleType.PERSON, ObstacleBehavior.SEMI_PREDICTABLE

    if radius < 0.9:
        return ObstacleType.CART, ObstacleBehavior.SEMI_PREDICTABLE

    return ObstacleType.UNKNOWN, ObstacleBehavior.UNPREDICTABLE


# ===================== TRACKER NODE =======================

class DynamicObstacleTracker(Node):
    """
    Subscribes to /scan, extracts dynamic obstacles,
    tracks them with Kalman filters, and publishes
    visualization markers and obstacle JSON for Nav2.
    """

    def __init__(self):
        super().__init__('dynamic_obstacle_tracker')

        # Parameters
        self.declare_parameter('scan_topic',       '/scan')
        self.declare_parameter('odom_topic',       '/odom')
        self.declare_parameter('cluster_eps',       0.30)
        self.declare_parameter('cluster_min_pts',   3)
        self.declare_parameter('max_range',         5.0)
        self.declare_parameter('min_cluster_radius', 0.10)
        self.declare_parameter('max_cluster_radius', 1.50)
        self.declare_parameter('prediction_horizon', 2.0)
        self.declare_parameter('publish_markers',  True)

        scan_topic     = self.get_parameter('scan_topic').value
        odom_topic     = self.get_parameter('odom_topic').value
        self._eps      = self.get_parameter('cluster_eps').value
        self._min_pts  = self.get_parameter('cluster_min_pts').value
        self._max_range = self.get_parameter('max_range').value
        self._min_r    = self.get_parameter('min_cluster_radius').value
        self._max_r    = self.get_parameter('max_cluster_radius').value
        self._pred_t   = self.get_parameter('prediction_horizon').value
        self._pub_markers = self.get_parameter('publish_markers').value

        # Robot pose (from odometry)
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0

        # Tracked obstacles
        self._obstacles: dict[int, TrackedObstacle] = {}
        self._next_id    = 0
        self._lock       = threading.Lock()
        self._last_time  = None

        # Association threshold
        self._assoc_threshold = 0.8  # meters

        # Subscriptions
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )

        # Publishers
        self._marker_pub = self.create_publisher(
            MarkerArray, '/obstacle_markers', 10
        )
        self._obstacle_pub = self.create_publisher(
            String, '/tracked_obstacles', 10
        )

        # Cleanup timer
        self._cleanup_timer = self.create_timer(0.5, self._cleanup_stale)

        self.get_logger().info('Dynamic Obstacle Tracker started')

    # ==================== CALLBACKS =======================

    def _odom_callback(self, msg: Odometry):
        """Update robot pose."""
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _scan_callback(self, msg: LaserScan):
        """Process LiDAR scan: cluster -> track -> publish."""
        now = self.get_clock().now().nanoseconds / 1e9
        dt  = (now - self._last_time) if self._last_time else 0.1
        self._last_time = now

        # Convert scan to Cartesian points (robot frame)
        points = self._scan_to_points(msg)

        if len(points) < self._min_pts:
            return

        # Cluster
        clusters = dbscan_cluster(points, eps=self._eps, min_pts=self._min_pts)

        # Get cluster centroids and radii
        detections = []
        for cluster in clusters:
            cx = float(np.mean(cluster[:, 0]))
            cy = float(np.mean(cluster[:, 1]))

            # Radius = max distance from centroid
            dists = np.sqrt(
                (cluster[:, 0] - cx) ** 2 +
                (cluster[:, 1] - cy) ** 2
            )
            radius = float(np.max(dists))

            if self._min_r <= radius <= self._max_r:
                # Convert to map frame
                mx, my = self._robot_to_map(cx, cy)
                detections.append((mx, my, radius))

        # Associate detections with tracked obstacles (nearest-neighbor)
        with self._lock:
            self._predict_all(dt)
            assigned = self._associate_and_update(detections)
            self._handle_unassigned(detections, assigned)

        # Publish
        self._publish_obstacles()
        if self._pub_markers:
            self._publish_markers()

    # ==================== SCAN PROCESSING =================

    def _scan_to_points(self, msg: LaserScan) -> np.ndarray:
        """Convert LaserScan to Cartesian point array (robot frame)."""
        points = []
        angle  = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= min(msg.range_max, self._max_range):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += msg.angle_increment
        return np.array(points) if points else np.zeros((0, 2))

    def _robot_to_map(self, rx: float, ry: float) -> tuple[float, float]:
        """Transform point from robot frame to map frame."""
        cos_y = math.cos(self._robot_yaw)
        sin_y = math.sin(self._robot_yaw)
        mx = self._robot_x + rx * cos_y - ry * sin_y
        my = self._robot_y + rx * sin_y + ry * cos_y
        return (mx, my)

    # ==================== TRACKING =======================

    def _predict_all(self, dt: float):
        """Predict all tracked obstacles forward."""
        for obs in self._obstacles.values():
            obs.kf.predict(dt)

    def _associate_and_update(
        self,
        detections: list[tuple[float, float, float]]
    ) -> set[int]:
        """
        Nearest-neighbor data association.
        Returns set of detection indices that were assigned.
        """
        assigned_detections = set()

        for obs in self._obstacles.values():
            best_dist = self._assoc_threshold
            best_det  = -1

            for i, (dx, dy, _) in enumerate(detections):
                if i in assigned_detections:
                    continue
                px, py = obs.kf.position
                dist = math.sqrt((dx - px) ** 2 + (dy - py) ** 2)
                if dist < best_dist:
                    best_dist = dist
                    best_det  = i

            if best_det >= 0:
                dx, dy, radius = detections[best_det]
                obs.kf.update(np.array([dx, dy]))
                obs.hits   += 1
                obs.misses  = 0
                obs.radius  = radius
                obs.last_seen = time.time()

                # Reclassify periodically
                obs.obs_type, obs.behavior = classify_obstacle(
                    np.array([[dx, dy]]), obs.kf.speed, radius
                )
                assigned_detections.add(best_det)
            else:
                obs.misses += 1

        return assigned_detections

    def _handle_unassigned(
        self,
        detections: list[tuple[float, float, float]],
        assigned: set[int]
    ):
        """Create new tracked obstacles for unassigned detections."""
        for i, (dx, dy, radius) in enumerate(detections):
            if i not in assigned:
                obs_id = self._next_id
                self._next_id += 1

                kf = KalmanFilter2D(dx, dy)
                obs = TrackedObstacle(
                    obstacle_id = obs_id,
                    kf          = kf,
                    radius      = radius
                )
                self._obstacles[obs_id] = obs

    def _cleanup_stale(self):
        """Remove obstacles not seen recently."""
        with self._lock:
            stale_ids = [
                oid for oid, obs in self._obstacles.items()
                if obs.stale
            ]
            for oid in stale_ids:
                del self._obstacles[oid]

    # ==================== PUBLISHING ======================

    def _publish_obstacles(self):
        """Publish confirmed tracked obstacles as JSON."""
        with self._lock:
            obs_list = []
            for obs in self._obstacles.values():
                if not obs.confirmed:
                    continue
                px, py = obs.kf.position
                vx, vy = obs.kf.velocity
                fpx, fpy = obs.kf.predict_position(self._pred_t)

                obs_list.append({
                    "id":            obs.obstacle_id,
                    "x":             round(px, 3),
                    "y":             round(py, 3),
                    "vx":            round(vx, 3),
                    "vy":            round(vy, 3),
                    "speed":         round(obs.kf.speed, 3),
                    "radius":        round(obs.radius, 3),
                    "type":          obs.obs_type.name,
                    "behavior":      obs.behavior.name,
                    "predicted_x":   round(fpx, 3),
                    "predicted_y":   round(fpy, 3),
                    "age":           round(obs.age_seconds, 1),
                })

        msg = String()
        msg.data = json.dumps({
            "obstacles":  obs_list,
            "count":      len(obs_list),
            "timestamp":  time.time()
        })
        self._obstacle_pub.publish(msg)

    def _publish_markers(self):
        """Publish RViz2 visualization markers for tracked obstacles."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clear old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        with self._lock:
            for obs in self._obstacles.values():
                if not obs.confirmed:
                    continue

                px, py = obs.kf.position

                # Obstacle sphere
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp    = now
                m.ns              = "obstacles"
                m.id              = obs.obstacle_id
                m.type            = Marker.CYLINDER
                m.action          = Marker.ADD
                m.pose.position.x = px
                m.pose.position.y = py
                m.pose.position.z = 0.9
                m.pose.orientation.w = 1.0
                m.scale.x = obs.radius * 2.0
                m.scale.y = obs.radius * 2.0
                m.scale.z = 1.8

                # Color by type
                color_map = {
                    ObstacleType.PERSON: (1.0, 0.3, 0.0, 0.7),
                    ObstacleType.CART:   (0.0, 0.6, 1.0, 0.7),
                    ObstacleType.STATIC: (0.5, 0.5, 0.5, 0.4),
                    ObstacleType.ROBOT:  (0.0, 1.0, 0.5, 0.7),
                    ObstacleType.UNKNOWN: (1.0, 1.0, 0.0, 0.6),
                }
                r, g, b, a = color_map.get(obs.obs_type, (1.0, 1.0, 0.0, 0.6))
                m.color.r = r
                m.color.g = g
                m.color.b = b
                m.color.a = a
                m.lifetime.sec = 1
                marker_array.markers.append(m)

                # Velocity arrow
                vx, vy = obs.kf.velocity
                if obs.kf.speed > 0.1:
                    arrow = Marker()
                    arrow.header.frame_id = "map"
                    arrow.header.stamp    = now
                    arrow.ns              = "velocities"
                    arrow.id              = obs.obstacle_id + 10000
                    arrow.type            = Marker.ARROW
                    arrow.action          = Marker.ADD

                    p_start = Point(x=px, y=py, z=1.0)
                    p_end   = Point(
                        x=px + vx * 1.5,
                        y=py + vy * 1.5,
                        z=1.0
                    )
                    arrow.points = [p_start, p_end]
                    arrow.scale.x = 0.05
                    arrow.scale.y = 0.10
                    arrow.scale.z = 0.10
                    arrow.color.r = 1.0
                    arrow.color.g = 0.0
                    arrow.color.b = 0.0
                    arrow.color.a = 0.9
                    arrow.lifetime.sec = 1
                    marker_array.markers.append(arrow)

                # Predicted position marker
                fpx, fpy = obs.kf.predict_position(self._pred_t)
                pred = Marker()
                pred.header.frame_id = "map"
                pred.header.stamp    = now
                pred.ns              = "predictions"
                pred.id              = obs.obstacle_id + 20000
                pred.type            = Marker.SPHERE
                pred.action          = Marker.ADD
                pred.pose.position.x = fpx
                pred.pose.position.y = fpy
                pred.pose.position.z = 0.5
                pred.pose.orientation.w = 1.0
                pred.scale.x = 0.2
                pred.scale.y = 0.2
                pred.scale.z = 0.2
                pred.color.r = 1.0
                pred.color.g = 0.5
                pred.color.b = 0.0
                pred.color.a = 0.5
                pred.lifetime.sec = 1
                marker_array.markers.append(pred)

        self._marker_pub.publish(marker_array)


# ===================== MAIN ===============================

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
