#!/usr/bin/env python3
"""
Frontier-Based Exploration
===========================
Autonomous exploration of the hospital using frontier detection.
Integrates with SLAM Toolbox for continuous map building.

Algorithm:
  1. Subscribe to occupancy grid from SLAM Toolbox
  2. Detect frontier cells (free cells adjacent to unknown cells)
  3. Cluster frontiers into regions
  4. Score frontiers: information gain - travel cost
  5. Send best frontier as Nav2 goal
  6. Repeat until map complete or task received

ROS 2 Jazzy | SLAM Toolbox | Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool

import math
import time
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


# ===================== CONSTANTS ==========================

FREE_THRESHOLD    = 20   # Cells <= this are free
UNKNOWN_VALUE     = -1   # Unknown cell value
OCCUPIED_THRESHOLD = 65  # Cells >= this are occupied

MIN_FRONTIER_SIZE = 5    # Minimum cells to consider a frontier
CLUSTER_RADIUS    = 1.5  # Meters - group frontiers within this radius


# ===================== DATA CLASSES =======================

@dataclass
class Frontier:
    centroid_x:   float
    centroid_y:   float
    size:         int    # Number of cells
    score:        float  = 0.0
    distance:     float  = 0.0


# ===================== EXPLORATION NODE ===================

class FrontierExplorer(Node):
    """
    Implements frontier-based exploration for hospital mapping.
    Stops when external delivery task arrives.
    """

    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.declare_parameter('map_topic',        '/map')
        self.declare_parameter('odom_topic',       '/odom')
        self.declare_parameter('exploration_rate', 1.0)
        self.declare_parameter('min_frontier_size', MIN_FRONTIER_SIZE)
        self.declare_parameter('robot_radius',     0.22)
        self.declare_parameter('gain_weight',      1.0)
        self.declare_parameter('cost_weight',      0.5)
        self.declare_parameter('max_frontier_dist', 15.0)

        self._min_size  = self.get_parameter('min_frontier_size').value
        self._robot_r   = self.get_parameter('robot_radius').value
        self._gain_w    = self.get_parameter('gain_weight').value
        self._cost_w    = self.get_parameter('cost_weight').value
        self._max_dist  = self.get_parameter('max_frontier_dist').value

        # State
        self._map: Optional[OccupancyGrid] = None
        self._map_lock  = threading.Lock()
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._exploring = False
        self._paused    = False  # Paused when delivery task active
        self._current_goal: Optional[Frontier] = None
        self._goal_handle  = None
        self._last_nav_time = 0.0
        self._explored_cells_count = 0

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._map_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
        )

        self._odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._odom_callback,
            10
        )

        self._pause_sub = self.create_subscription(
            Bool,
            '/exploration/pause',
            self._pause_callback,
            10
        )

        # Publishers
        self._frontier_pub = self.create_publisher(
            MarkerArray, '/frontier_markers', 10
        )
        self._status_pub = self.create_publisher(
            String, '/exploration/status', 10
        )

        # Exploration timer
        rate = self.get_parameter('exploration_rate').value
        self._explore_timer = self.create_timer(
            1.0 / rate,
            self._explore_step
        )

        self.get_logger().info('Frontier Explorer started')

    # ==================== CALLBACKS =======================

    def _map_callback(self, msg: OccupancyGrid):
        with self._map_lock:
            self._map = msg

    def _odom_callback(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _pause_callback(self, msg: Bool):
        self._paused = msg.data
        if self._paused:
            self.get_logger().info('Exploration PAUSED - delivery task active')
            self._cancel_current_goal()
        else:
            self.get_logger().info('Exploration RESUMED')

    # ==================== FRONTIER DETECTION ==============

    def _get_map_array(self) -> tuple[Optional[np.ndarray], Optional[OccupancyGrid]]:
        """Return occupancy grid as numpy array."""
        with self._map_lock:
            if self._map is None:
                return None, None
            map_copy = self._map

        data = np.array(map_copy.data, dtype=np.int8).reshape(
            map_copy.info.height,
            map_copy.info.width
        )
        return data, map_copy

    def _detect_frontiers(self,
                           grid: np.ndarray,
                           info: OccupancyGrid) -> list[tuple[float, float]]:
        """
        Detect frontier cells: free cells adjacent to unknown cells.
        Returns list of (world_x, world_y) frontier points.
        """
        h, w = grid.shape
        frontier_pts = []

        # Use vectorized approach for speed
        # Pad grid to avoid boundary checks
        padded = np.pad(grid, 1, constant_values=UNKNOWN_VALUE)

        for r in range(1, h + 1):
            for c in range(1, w + 1):
                cell = padded[r, c]
                if cell > FREE_THRESHOLD or cell == UNKNOWN_VALUE:
                    continue  # Not a free cell

                # Check 8-neighbors for unknown
                neighbors = [
                    padded[r-1, c-1], padded[r-1, c], padded[r-1, c+1],
                    padded[r,   c-1],                  padded[r,   c+1],
                    padded[r+1, c-1], padded[r+1, c], padded[r+1, c+1],
                ]

                if UNKNOWN_VALUE in neighbors:
                    # Convert cell to world coordinates
                    wx = (info.info.origin.position.x +
                          (c - 1) * info.info.resolution +
                          info.info.resolution / 2.0)
                    wy = (info.info.origin.position.y +
                          (r - 1) * info.info.resolution +
                          info.info.resolution / 2.0)
                    frontier_pts.append((wx, wy))

        return frontier_pts

    def _cluster_frontiers(
        self,
        points: list[tuple[float, float]]
    ) -> list[Frontier]:
        """
        Cluster frontier points into regions using simple greedy clustering.
        Returns scored Frontier objects.
        """
        if not points:
            return []

        pts = np.array(points)
        n   = len(pts)
        assigned = np.full(n, -1, dtype=int)
        cluster_id = 0
        clusters: list[list[int]] = []

        for i in range(n):
            if assigned[i] >= 0:
                continue
            cluster = [i]
            assigned[i] = cluster_id
            # Find all points within cluster radius
            for j in range(i + 1, n):
                if assigned[j] >= 0:
                    continue
                dx = pts[i, 0] - pts[j, 0]
                dy = pts[i, 1] - pts[j, 1]
                if dx * dx + dy * dy <= CLUSTER_RADIUS ** 2:
                    cluster.append(j)
                    assigned[j] = cluster_id
            clusters.append(cluster)
            cluster_id += 1

        frontiers = []
        for cluster in clusters:
            if len(cluster) < self._min_size:
                continue
            cx = float(np.mean(pts[cluster, 0]))
            cy = float(np.mean(pts[cluster, 1]))
            size = len(cluster)
            dist = math.sqrt(
                (cx - self._robot_x) ** 2 +
                (cy - self._robot_y) ** 2
            )
            if dist > self._max_dist:
                continue
            # Score = gain / cost
            gain = size  # Information gain proportional to frontier size
            cost = max(dist, 0.1)
            score = (self._gain_w * gain) - (self._cost_w * cost)
            frontiers.append(Frontier(
                centroid_x = cx,
                centroid_y = cy,
                size       = size,
                score      = score,
                distance   = dist
            ))

        # Sort by score descending
        frontiers.sort(key=lambda f: f.score, reverse=True)
        return frontiers

    # ==================== EXPLORATION LOOP ================

    def _explore_step(self):
        """Main exploration tick: detect frontiers and send next goal."""
        if self._paused:
            return

        # Don't send new goal if Nav2 is still processing
        if self._current_goal and (time.time() - self._last_nav_time) < 15.0:
            return

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            return

        grid, info = self._get_map_array()
        if grid is None:
            return

        # Detect and cluster frontiers
        raw_frontiers = self._detect_frontiers(grid, info)

        if not raw_frontiers:
            self.get_logger().info(
                'No frontiers found. Map may be complete.'
            )
            self._publish_status("COMPLETE", 0)
            return

        frontiers = self._cluster_frontiers(raw_frontiers)

        if not frontiers:
            return

        best = frontiers[0]
        self.get_logger().info(
            f'Navigating to frontier: ({best.centroid_x:.1f}, '
            f'{best.centroid_y:.1f}) size={best.size} score={best.score:.1f}'
        )

        self._send_frontier_goal(best)
        self._publish_frontier_markers(frontiers)
        self._publish_status("EXPLORING", len(frontiers))

    def _send_frontier_goal(self, frontier: Frontier):
        """Send frontier centroid to Nav2."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = frontier.centroid_x
        goal.pose.pose.position.y = frontier.centroid_y
        goal.pose.pose.orientation.w = 1.0

        self._current_goal  = frontier
        self._last_nav_time = time.time()

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._frontier_goal_response)

    def _frontier_goal_response(self, future):
        """Handle Nav2 goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Frontier goal rejected')
            self._current_goal = None
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._frontier_result)

    def _frontier_result(self, future):
        """Handle frontier navigation result."""
        result = future.result()
        self._current_goal = None
        self._goal_handle  = None
        if result.status == 4:
            self.get_logger().info('Frontier reached. Exploring further.')
        else:
            self.get_logger().warn(
                f'Frontier navigation failed (status: {result.status})'
            )

    def _cancel_current_goal(self):
        """Cancel active frontier goal."""
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self._current_goal = None
            self._goal_handle  = None

    # ==================== PUBLISHING ======================

    def _publish_frontier_markers(self, frontiers: list[Frontier]):
        """Visualize frontiers in RViz2."""
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, f in enumerate(frontiers[:20]):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = now
            m.ns     = 'frontiers'
            m.id     = i
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f.centroid_x
            m.pose.position.y = f.centroid_y
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            # Best frontier = green, others = yellow->red
            ratio = i / max(len(frontiers) - 1, 1)
            m.color.r = ratio
            m.color.g = 1.0 - ratio * 0.5
            m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime.sec = 2
            arr.markers.append(m)

        self._frontier_pub.publish(arr)

    def _publish_status(self, state: str, frontier_count: int):
        """Publish exploration status."""
        import json
        msg = String()
        msg.data = json.dumps({
            "state":           state,
            "frontier_count":  frontier_count,
            "robot_pos":       [round(self._robot_x, 2), round(self._robot_y, 2)],
            "timestamp":       time.time()
        })
        self._status_pub.publish(msg)


# ===================== MAIN ===============================

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
