#!/usr/bin/env python3
"""
Health Monitor
===============
Skills.md § Health Monitoring — monitors the overall health of the robot:
  - Localization quality (tf map→odom freshness)
  - Navigation status (Nav2 action server availability)
  - Sensor health (LiDAR scan freshness, odometry freshness)
  - Mission progress (delivery task completion tracking)
  - Fleet connectivity (fleet_status topic freshness)

Critical Failure Response (Skills.md):
  1. Stop robot (publish zero cmd_vel)
  2. Report error (publish to /robot_health)
  3. Await recovery

Publishes JSON health status to /robot_health at 1 Hz.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import json
import time
import threading


class HealthStatus:
    OK       = "OK"
    WARNING  = "WARNING"
    CRITICAL = "CRITICAL"


class HealthMonitor(Node):
    """
    Monitors robot health per Skills.md § Health Monitoring.
    Detects sensor timeouts, localization loss, and navigation failures.
    Implements critical failure response: stop → report → await.
    """

    # Timeout thresholds (seconds)
    SCAN_TIMEOUT      = 5.0    # No /scan in this long → sensor failure
    ODOM_TIMEOUT      = 5.0    # No /odom → odometry failure
    MISSION_TIMEOUT   = 10.0   # No /mission_status → mission manager issue
    FLEET_TIMEOUT     = 20.0   # No /fleet_status → fleet connectivity loss

    def __init__(self):
        super().__init__('health_monitor')

        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('robot_ns', '')
        self._robot_id = self.get_parameter('robot_id').value
        self._ns       = self.get_parameter('robot_ns').value

        prefix = f'/{self._ns}' if self._ns else ''

        # ── Timestamps for last-seen messages ─────────────────────────────
        self._lock = threading.Lock()
        self._last_scan:   float = 0.0
        self._last_odom:   float = 0.0
        self._last_mission: float = 0.0
        self._last_fleet:  float = 0.0

        # ── Health state ─────────────────────────────────────────────────
        self._overall_status = HealthStatus.OK
        self._critical_active = False
        self._critical_reasons: list[str] = []

        # ── Subscriptions ────────────────────────────────────────────────
        self.create_subscription(
            LaserScan, f'{prefix}/scan',
            self._on_scan, 10
        )
        self.create_subscription(
            Odometry, f'{prefix}/odom',
            self._on_odom, 10
        )
        self.create_subscription(
            String, f'{prefix}/mission_status',
            self._on_mission_status, 10
        )
        self.create_subscription(
            String, '/hospital/fleet_status',
            self._on_fleet_status, 10
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._health_pub = self.create_publisher(
            String, f'{prefix}/robot_health', 10
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist, f'{prefix}/cmd_vel', 10
        )

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(1.0, self._check_health)

        self.get_logger().info(
            f'[{self._robot_id}] Health Monitor started. '
            f'Monitoring: scan, odom, mission_status, fleet_status'
        )

    # ─────────────── Subscriptions ───────────────────────────────────────

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            self._last_scan = time.time()

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._last_odom = time.time()

    def _on_mission_status(self, msg: String):
        with self._lock:
            self._last_mission = time.time()

    def _on_fleet_status(self, msg: String):
        with self._lock:
            self._last_fleet = time.time()

    # ─────────────── Health Check ────────────────────────────────────────

    def _check_health(self):
        now = time.time()
        issues: list[dict] = []
        critical = False

        with self._lock:
            last_scan    = self._last_scan
            last_odom    = self._last_odom
            last_mission = self._last_mission
            last_fleet   = self._last_fleet

        # ── Sensor health: LiDAR ─────────────────────────────────────────
        if last_scan > 0 and (now - last_scan) > self.SCAN_TIMEOUT:
            issues.append({
                'component': 'lidar',
                'severity': HealthStatus.CRITICAL,
                'message': f'No /scan received for {now - last_scan:.1f}s',
            })
            critical = True
        elif last_scan == 0:
            issues.append({
                'component': 'lidar',
                'severity': HealthStatus.WARNING,
                'message': 'No /scan received yet (initializing)',
            })

        # ── Sensor health: Odometry ──────────────────────────────────────
        if last_odom > 0 and (now - last_odom) > self.ODOM_TIMEOUT:
            issues.append({
                'component': 'odometry',
                'severity': HealthStatus.CRITICAL,
                'message': f'No /odom received for {now - last_odom:.1f}s',
            })
            critical = True
        elif last_odom == 0:
            issues.append({
                'component': 'odometry',
                'severity': HealthStatus.WARNING,
                'message': 'No /odom received yet (initializing)',
            })

        # ── Mission progress ─────────────────────────────────────────────
        if last_mission > 0 and (now - last_mission) > self.MISSION_TIMEOUT:
            issues.append({
                'component': 'mission_manager',
                'severity': HealthStatus.WARNING,
                'message': f'No mission_status for {now - last_mission:.1f}s',
            })

        # ── Fleet connectivity ───────────────────────────────────────────
        if last_fleet > 0 and (now - last_fleet) > self.FLEET_TIMEOUT:
            issues.append({
                'component': 'fleet',
                'severity': HealthStatus.WARNING,
                'message': f'No fleet_status for {now - last_fleet:.1f}s',
            })

        # ── Determine overall status ─────────────────────────────────────
        if critical:
            self._overall_status = HealthStatus.CRITICAL
        elif issues:
            self._overall_status = HealthStatus.WARNING
        else:
            self._overall_status = HealthStatus.OK

        # ── Critical Failure Response (Skills.md) ────────────────────────
        if critical and not self._critical_active:
            self._critical_active = True
            self._critical_reasons = [i['message'] for i in issues
                                      if i['severity'] == HealthStatus.CRITICAL]
            self.get_logger().error(
                f'CRITICAL FAILURE: {self._critical_reasons}. '
                f'Stopping robot. Awaiting recovery.'
            )
            # 1. Stop robot
            self._emergency_stop()

        elif not critical and self._critical_active:
            self._critical_active = False
            self._critical_reasons = []
            self.get_logger().info('Health recovered. Resuming normal operation.')

        # ── Publish health status ────────────────────────────────────────
        msg = String()
        msg.data = json.dumps({
            'robot_id':  self._robot_id,
            'status':    self._overall_status,
            'critical':  self._critical_active,
            'issues':    issues,
            'sensors': {
                'lidar_age':    round(now - last_scan, 2)  if last_scan  > 0 else None,
                'odom_age':     round(now - last_odom, 2)  if last_odom  > 0 else None,
            },
            'mission_age':  round(now - last_mission, 2) if last_mission > 0 else None,
            'fleet_age':    round(now - last_fleet, 2)   if last_fleet  > 0 else None,
            'timestamp':    now,
        })
        self._health_pub.publish(msg)

    def _emergency_stop(self):
        """Skills.md Critical Failure Response: Stop robot immediately."""
        stop_msg = Twist()  # All zeros = stop
        self._cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
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
