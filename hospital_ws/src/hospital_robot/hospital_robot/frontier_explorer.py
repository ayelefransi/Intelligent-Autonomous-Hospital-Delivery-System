#!/usr/bin/env python3
"""
Frontier Explorer
==================
Autonomous SLAM-driven exploration of the hospital.
Detects frontiers (free cells bordering unknown space),
clusters them, scores by information gain vs travel cost,
and sends the best frontier as a Nav2 goal.

Pauses automatically when a delivery task is active.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool

import json
import math
import time
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


FREE_THRESH    = 20
UNKNOWN_VAL    = -1
OCC_THRESH     = 65
MIN_FRONTIER   = 4
CLUSTER_RADIUS = 1.2   # meters


@dataclass
class Frontier:
    cx:      float
    cy:      float
    size:    int
    dist:    float
    score:   float = 0.0


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic',   '/map')
        self.declare_parameter('explore_hz',   1.0)
        self.declare_parameter('max_dist',    15.0)
        self.declare_parameter('gain_w',       1.0)
        self.declare_parameter('cost_w',       0.5)
        self.declare_parameter('goal_timeout', 20.0)

        self._max_dist    = self.get_parameter('max_dist').value
        self._gain_w      = self.get_parameter('gain_w').value
        self._cost_w      = self.get_parameter('cost_w').value
        self._goal_timeout = self.get_parameter('goal_timeout').value

        self._map:    Optional[OccupancyGrid] = None
        self._map_lock = threading.Lock()
        self._rx = self._ry = 0.0
        self._paused       = False
        self._active_goal  = None
        self._goal_handle  = None
        self._last_nav     = 0.0

        # Nav2
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Map subscription (transient local for latching)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._map_cb,
            map_qos
        )
        self.create_subscription(Odometry,  '/odom',           self._odom_cb,  10)
        self.create_subscription(Bool,      '/explore/pause',  self._pause_cb, 10)

        self._frontier_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self._status_pub   = self.create_publisher(String,      '/explore/status',   10)

        hz = self.get_parameter('explore_hz').value
        self.create_timer(1.0 / hz, self._step)

        self.get_logger().info('Frontier Explorer started')

    # ─────────────── Callbacks ───────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        with self._map_lock:
            self._map = msg

    def _odom_cb(self, msg: Odometry):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y

    def _pause_cb(self, msg: Bool):
        self._paused = msg.data
        if self._paused:
            self.get_logger().info('Exploration PAUSED')
            self._cancel()
        else:
            self.get_logger().info('Exploration RESUMED')

    # ─────────────── Exploration step ────────────────────────────────────────

    def _step(self):
        if self._paused:
            return

        # Wait for current goal to finish or timeout
        elapsed = time.time() - self._last_nav
        if self._active_goal and elapsed < self._goal_timeout:
            return

        # Timeout: abandon old goal and try new frontier
        if self._active_goal and elapsed >= self._goal_timeout:
            self.get_logger().warn('Frontier goal timed out, picking new one')
            self._cancel()

        if not self._nav.wait_for_server(timeout_sec=1.0):
            return

        with self._map_lock:
            if self._map is None:
                return
            grid = np.array(self._map.data, dtype=np.int8).reshape(
                self._map.info.height, self._map.info.width
            )
            info = self._map.info

        raw = self._detect_frontiers(grid, info)
        if not raw:
            self.get_logger().info('No frontiers. Exploration complete.')
            self._publish_status('COMPLETE', 0)
            return

        frontiers = self._cluster(raw)
        if not frontiers:
            return

        best = frontiers[0]
        self.get_logger().info(
            f'Frontier: ({best.cx:.1f},{best.cy:.1f}) '
            f'size={best.size} dist={best.dist:.1f}m score={best.score:.1f}'
        )

        self._send_goal(best)
        self._publish_markers(frontiers)
        self._publish_status('EXPLORING', len(frontiers))

    # ─────────────── Frontier detection ──────────────────────────────────────

    def _detect_frontiers(self, grid: np.ndarray,
                          info) -> list[tuple[float, float]]:
        h, w = grid.shape
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y
        pts  = []

        # Vectorised row scan
        for r in range(1, h - 1):
            for c in range(1, w - 1):
                v = int(grid[r, c])
                if v > FREE_THRESH or v == UNKNOWN_VAL:
                    continue
                # 4-connected neighbors contain unknown?
                nb = [int(grid[r-1,c]), int(grid[r+1,c]),
                      int(grid[r,c-1]), int(grid[r,c+1])]
                if UNKNOWN_VAL in nb:
                    wx = ox + (c + 0.5) * res
                    wy = oy + (r + 0.5) * res
                    pts.append((wx, wy))
        return pts

    # ─────────────── Clustering ──────────────────────────────────────────────

    def _cluster(self, pts: list[tuple[float, float]]) -> list[Frontier]:
        if not pts:
            return []
        arr      = np.array(pts)
        n        = len(arr)
        assigned = np.full(n, -1, dtype=int)
        cid      = 0
        clusters: list[list[int]] = []

        for i in range(n):
            if assigned[i] >= 0:
                continue
            d = np.sqrt(((arr - arr[i])**2).sum(1))
            member = np.where(d <= CLUSTER_RADIUS)[0]
            for idx in member:
                if assigned[idx] < 0:
                    assigned[idx] = cid
            clusters.append(member.tolist())
            cid += 1

        result = []
        for cl in clusters:
            if len(cl) < MIN_FRONTIER:
                continue
            cx   = float(arr[cl, 0].mean())
            cy   = float(arr[cl, 1].mean())
            dist = math.sqrt((cx - self._rx)**2 + (cy - self._ry)**2)
            if dist > self._max_dist or dist < 0.3:
                continue
            gain  = len(cl)
            score = self._gain_w * gain - self._cost_w * dist
            result.append(Frontier(cx=cx, cy=cy, size=len(cl),
                                   dist=dist, score=score))

        result.sort(key=lambda f: f.score, reverse=True)
        return result

    # ─────────────── Nav2 goal ───────────────────────────────────────────────

    def _send_goal(self, f: Frontier):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = f.cx
        goal.pose.pose.position.y = f.cy
        goal.pose.pose.orientation.w = 1.0
        self._active_goal = f
        self._last_nav    = time.time()
        fut = self._nav.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('Frontier goal rejected')
            self._active_goal = None
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, fut):
        self._active_goal = None
        self._goal_handle = None
        status = fut.result().status
        if status == 4:
            self.get_logger().info('Frontier reached')
        else:
            self.get_logger().warn(f'Frontier nav failed: status={status}')

    def _cancel(self):
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
        self._active_goal = None
        self._goal_handle = None

    # ─────────────── Publishers ───────────────────────────────────────────────

    def _publish_markers(self, frontiers: list[Frontier]):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        clr = Marker(); clr.action = Marker.DELETEALL
        arr.markers.append(clr)
        for i, f in enumerate(frontiers[:25]):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = now
            m.ns     = 'frontiers'
            m.id     = i
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f.cx
            m.pose.position.y = f.cy
            m.pose.position.z = 0.4
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            ratio    = i / max(len(frontiers) - 1, 1)
            m.color.r = ratio
            m.color.g = 1.0 - ratio * 0.5
            m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime.sec = 3
            arr.markers.append(m)
        self._frontier_pub.publish(arr)

    def _publish_status(self, state: str, n: int):
        msg      = String()
        msg.data = json.dumps({
            'state':    state,
            'frontiers': n,
            'pos':      [round(self._rx, 2), round(self._ry, 2)],
            'ts':       time.time(),
        })
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
