#!/usr/bin/env python3
"""
Dynamic Obstacle Tracker
=========================
LiDAR scan -> DBSCAN clusters -> Kalman-filtered tracks -> RViz markers

Pipeline per scan:
  1. Convert LaserScan to 2D Cartesian (robot frame)
  2. DBSCAN: group into obstacle candidates
  3. Associate clusters to existing tracks (nearest-neighbor)
  4. Kalman predict + update per track
  5. Classify: PERSON / CART / STATIC
  6. Publish /tracked_obstacles (JSON) + /obstacle_markers (RViz)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String

import json
import math
import time
import threading
import numpy as np
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


# =============================================================================
# TYPES
# =============================================================================

class OType(Enum):
    UNKNOWN = 0
    DOCTOR  = 1   # < 0.5m radius, speed > 0.6 m/s — fast, unpredictable
    NURSE   = 2   # < 0.5m radius, speed 0.05-0.6 m/s — semi-predictable
    CART    = 3   # 0.5-0.9m radius, moving — predictable
    STATIC  = 4   # Not moving


# Robot response per Skills.md § Obstacle Classification
OBSTACLE_RESPONSE = {
    OType.DOCTOR:  'YIELD',               # Yield immediately, slow down, increase safety margin
    OType.NURSE:   'MAINTAIN_CLEARANCE',   # Maintain clearance, continue if safe
    OType.CART:    'REPLAN',               # Replan if required
    OType.STATIC:  'NAVIGATE_AROUND',      # Navigate around obstacle
    OType.UNKNOWN: 'MAINTAIN_CLEARANCE',   # Default: cautious
}


# =============================================================================
# KALMAN FILTER  state=[x, y, vx, vy]  meas=[x, y]
# =============================================================================

class KF:
    def __init__(self, x: float, y: float):
        self.s = np.array([x, y, 0.0, 0.0])
        self.P = np.eye(4)
        self.H = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
        self.Q = np.diag([0.1, 0.1, 0.5, 0.5])
        self.R = np.diag([0.05, 0.05])

    def predict(self, dt: float):
        F = np.eye(4); F[0,2] = dt; F[1,3] = dt
        self.s = F @ self.s
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z: np.ndarray):
        y  = z - self.H @ self.s
        S  = self.H @ self.P @ self.H.T + self.R
        K  = self.P @ self.H.T @ np.linalg.inv(S)
        self.s = self.s + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def pos(self):   return self.s[0], self.s[1]
    @property
    def vel(self):   return self.s[2], self.s[3]
    @property
    def speed(self): return math.sqrt(self.s[2]**2 + self.s[3]**2)

    def future_pos(self, t: float):
        return self.s[0] + self.s[2]*t, self.s[1] + self.s[3]*t


@dataclass
class Track:
    tid:        int
    kf:         KF
    otype:      OType = OType.UNKNOWN
    radius:     float = 0.3
    hits:       int   = 0
    misses:     int   = 0
    born:       float = field(default_factory=time.time)
    last_seen:  float = field(default_factory=time.time)

    @property
    def confirmed(self): return self.hits >= 3
    @property
    def dead(self):      return self.misses >= 5


# =============================================================================
# DBSCAN  (pure numpy, no sklearn)
# =============================================================================

def dbscan(pts: np.ndarray, eps: float = 0.3, min_pts: int = 3):
    n = len(pts)
    if n == 0:
        return []
    labels = np.full(n, -1, dtype=int)
    cid = 0

    def nb(i):
        d = np.sqrt(((pts - pts[i])**2).sum(1))
        return np.where(d <= eps)[0].tolist()

    for i in range(n):
        if labels[i] != -1:
            continue
        nbs = nb(i)
        if len(nbs) < min_pts:
            labels[i] = -2
            continue
        labels[i] = cid
        seed = list(nbs)
        k = 0
        while k < len(seed):
            q = seed[k]
            if labels[q] < 0:
                labels[q] = cid
                qnbs = nb(q)
                if len(qnbs) >= min_pts:
                    seed.extend(qnbs)
            k += 1
        cid += 1

    return [pts[labels == c] for c in range(cid) if (labels == c).sum() >= min_pts]


def classify(radius: float, speed: float) -> OType:
    """Classify obstacle per Skills.md § Obstacle Classification.

    Doctor:  fast (>0.6 m/s), unpredictable — small radius, high speed
    Nurse:   semi-predictable — small radius, moderate speed
    Cart:    predictable motion — larger radius
    Static:  not moving
    """
    if speed < 0.05:
        return OType.STATIC
    if radius < 0.5:
        if speed > 0.6:
            return OType.DOCTOR    # Fast, unpredictable
        return OType.NURSE         # Semi-predictable
    if radius < 0.9:
        return OType.CART
    return OType.UNKNOWN


# =============================================================================
# TRACKER NODE
# =============================================================================

class ObstacleTracker(Node):

    def __init__(self):
        super().__init__('obstacle_tracker')

        self.declare_parameter('scan_topic',    '/scan')
        self.declare_parameter('max_range',      4.0)
        self.declare_parameter('cluster_eps',    0.30)
        self.declare_parameter('cluster_minpts', 3)
        self.declare_parameter('min_radius',     0.10)
        self.declare_parameter('max_radius',     1.20)
        self.declare_parameter('pred_horizon',   2.0)
        self.declare_parameter('assoc_thresh',   0.80)

        self._max_r   = self.get_parameter('max_range').value
        self._eps     = self.get_parameter('cluster_eps').value
        self._minpts  = self.get_parameter('cluster_minpts').value
        self._rmin    = self.get_parameter('min_radius').value
        self._rmax    = self.get_parameter('max_radius').value
        self._pred_t  = self.get_parameter('pred_horizon').value
        self._assoc   = self.get_parameter('assoc_thresh').value

        self._rx = self._ry = self._ryaw = 0.0
        self._tracks: dict[int, Track] = {}
        self._next_id = 0
        self._last_t  = None
        self._lock    = threading.Lock()

        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self._marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self._track_pub  = self.create_publisher(String,      '/tracked_obstacles', 10)

        self.create_timer(0.5, self._cleanup)

        self.get_logger().info('Obstacle Tracker started')

    # ─────────────── Callbacks ───────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._ryaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y**2 + q.z**2)
        )

    def _scan_cb(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt  = (now - self._last_t) if self._last_t else 0.1
        self._last_t = now

        pts = self._to_cartesian(msg)
        if len(pts) < self._minpts:
            return

        clusters = dbscan(pts, eps=self._eps, min_pts=self._minpts)

        detections = []
        for c in clusters:
            cx = float(c[:,0].mean())
            cy = float(c[:,1].mean())
            r  = float(np.sqrt(((c - c.mean(0))**2).sum(1)).max())
            if self._rmin <= r <= self._rmax:
                mx, my = self._to_map(cx, cy)
                detections.append((mx, my, r))

        with self._lock:
            for t in self._tracks.values():
                t.kf.predict(dt)

            assigned = self._associate(detections)
            self._new_tracks(detections, assigned)

        self._publish_tracks()
        self._publish_markers()

    # ─────────────── Tracking ────────────────────────────────────────────────

    def _associate(self, dets):
        assigned = set()
        for t in self._tracks.values():
            px, py  = t.kf.pos
            best_d  = self._assoc
            best_i  = -1
            for i, (dx, dy, _) in enumerate(dets):
                if i in assigned:
                    continue
                d = math.sqrt((dx-px)**2 + (dy-py)**2)
                if d < best_d:
                    best_d = d; best_i = i
            if best_i >= 0:
                dx, dy, r = dets[best_i]
                t.kf.update(np.array([dx, dy]))
                t.hits  += 1
                t.misses = 0
                t.radius = r
                t.last_seen = time.time()
                t.otype = classify(r, t.kf.speed)
                assigned.add(best_i)
            else:
                t.misses += 1
        return assigned

    def _new_tracks(self, dets, assigned):
        for i, (dx, dy, r) in enumerate(dets):
            if i not in assigned:
                t = Track(
                    tid    = self._next_id,
                    kf     = KF(dx, dy),
                    radius = r,
                )
                self._tracks[self._next_id] = t
                self._next_id += 1

    def _cleanup(self):
        with self._lock:
            dead = [tid for tid, t in self._tracks.items() if t.dead]
            for tid in dead:
                del self._tracks[tid]

    # ─────────────── Helpers ─────────────────────────────────────────────────

    def _to_cartesian(self, msg: LaserScan) -> np.ndarray:
        pts = []
        ang = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= min(msg.range_max, self._max_r):
                pts.append([r*math.cos(ang), r*math.sin(ang)])
            ang += msg.angle_increment
        return np.array(pts) if pts else np.zeros((0, 2))

    def _to_map(self, rx: float, ry: float):
        c, s = math.cos(self._ryaw), math.sin(self._ryaw)
        return self._rx + rx*c - ry*s, self._ry + rx*s + ry*c

    # ─────────────── Publishing ───────────────────────────────────────────────

    def _publish_tracks(self):
        with self._lock:
            data = []
            for t in self._tracks.values():
                if not t.confirmed:
                    continue
                try:
                    px, py   = t.kf.pos
                    vx, vy   = t.kf.vel
                    fpx, fpy = t.kf.future_pos(self._pred_t)
                    data.append({
                        'id': t.tid, 'x': round(px,3), 'y': round(py,3),
                        'vx': round(vx,3), 'vy': round(vy,3),
                        'speed': round(t.kf.speed,3),
                        'radius': round(t.radius,3),
                        'type': t.otype.name,
                        'response': OBSTACLE_RESPONSE.get(t.otype, 'MAINTAIN_CLEARANCE'),
                        'pred_x': round(fpx,3), 'pred_y': round(fpy,3),
                    })
                except Exception:
                    continue
        msg      = String()
        msg.data = json.dumps({'obstacles': data, 'n': len(data), 'ts': time.time()})
        self._track_pub.publish(msg)

    def _publish_markers(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        clr = Marker()
        clr.action = Marker.DELETEALL
        arr.markers.append(clr)

        COLOR = {
            OType.DOCTOR:  (1.0, 0.3, 0.0, 0.7),   # orange — yield immediately
            OType.NURSE:   (0.9, 0.7, 0.0, 0.7),   # amber — maintain clearance
            OType.CART:    (0.0, 0.6, 1.0, 0.7),   # blue — replan
            OType.STATIC:  (0.5, 0.5, 0.5, 0.4),   # gray — navigate around
            OType.UNKNOWN: (1.0, 1.0, 0.0, 0.6),   # yellow
        }

        with self._lock:
            for t in self._tracks.values():
                if not t.confirmed:
                    continue
                px, py = t.kf.pos
                r, g, b, a = COLOR.get(t.otype, (1,1,0,0.6))

                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp    = now
                m.ns   = 'obstacles'
                m.id   = t.tid
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position.x = px
                m.pose.position.y = py
                m.pose.position.z = 0.9
                m.pose.orientation.w = 1.0
                m.scale.x = t.radius * 2
                m.scale.y = t.radius * 2
                m.scale.z = 1.8
                m.color.r = r; m.color.g = g
                m.color.b = b; m.color.a = a
                m.lifetime.sec = 1
                arr.markers.append(m)

                # Velocity arrow
                vx, vy = t.kf.vel
                if t.kf.speed > 0.1:
                    ar = Marker()
                    ar.header.frame_id = 'map'
                    ar.header.stamp    = now
                    ar.ns   = 'velocities'
                    ar.id   = t.tid + 10000
                    ar.type = Marker.ARROW
                    ar.action = Marker.ADD
                    p1 = Point(x=px, y=py, z=1.0)
                    p2 = Point(x=px+vx*1.5, y=py+vy*1.5, z=1.0)
                    ar.points = [p1, p2]
                    ar.scale.x = 0.05
                    ar.scale.y = 0.10
                    ar.scale.z = 0.10
                    ar.color.r = 1.0; ar.color.a = 0.9
                    ar.lifetime.sec = 1
                    arr.markers.append(ar)

        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTracker()
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
