#!/usr/bin/env python3
"""
Fleet Coordinator
==================
Manages 3 TurtleBot3 Waffle robots using Hungarian algorithm
for optimal task->robot assignment.

Architecture:
  - Listens to /hospital/request for new delivery jobs
  - Tracks robot positions via /robot_N/odom
  - Tracks robot availability via /robot_N/task_log
  - Assigns tasks via /robot_N/delivery_request
  - Publishes fleet status to /hospital/fleet_status

Cost matrix: (n_tasks x n_robots)
  cost[i][j] = dist(task_dest, robot_pos)
               x priority_weight
               x load_factor
               x (2 - success_rate)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import json
import math
import time
import threading
import numpy as np
from scipy.optimize import linear_sum_assignment
from dataclasses import dataclass, field
from typing import Optional

from hospital_robot.mission_manager import HOSPITAL_LOCATIONS


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class RobotState:
    robot_id:     str
    x:            float = 0.0
    y:            float = 0.0
    available:    bool  = True
    current_task: Optional[str] = None
    task_count:   int   = 0
    ok_count:     int   = 0
    last_seen:    float = field(default_factory=time.time)

    @property
    def success_rate(self) -> float:
        return (self.ok_count / self.task_count) if self.task_count else 1.0


@dataclass
class PendingTask:
    task_id:  str
    dest:     str
    dest_x:   float
    dest_y:   float
    priority: int
    payload:  str
    created:  float = field(default_factory=time.time)


# =============================================================================
# COORDINATOR NODE
# =============================================================================

class FleetCoordinator(Node):

    ROBOTS = ['robot_1', 'robot_2', 'robot_3']
    ROBOT_TIMEOUT = 15.0  # seconds

    def __init__(self):
        super().__init__('fleet_coordinator')

        self.declare_parameter('assign_hz', 2.0)
        rate = self.get_parameter('assign_hz').value

        self._robots:   dict[str, RobotState]  = {}
        self._pending:  list[PendingTask]       = []
        self._assigned: set[str]                = set()
        self._lock      = threading.Lock()

        # Register robots
        for rid in self.ROBOTS:
            self._robots[rid] = RobotState(robot_id=rid)

        # ── Subscriptions ────────────────────────────────────────────────────
        self._odom_subs = []
        for rid in self.ROBOTS:
            sub = self.create_subscription(
                Odometry, f'/{rid}/odom',
                lambda msg, r=rid: self._odom_cb(msg, r), 10
            )
            self._odom_subs.append(sub)

        self._status_subs = []
        for rid in self.ROBOTS:
            sub = self.create_subscription(
                String, f'/{rid}/task_log',
                lambda msg, r=rid: self._status_cb(msg, r), 10
            )
            self._status_subs.append(sub)

        self.create_subscription(
            String, '/hospital/request',
            self._request_cb, 10
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._dispatch_pubs = {}
        for rid in self.ROBOTS:
            self._dispatch_pubs[rid] = self.create_publisher(
                String, f'/{rid}/delivery_request', 10
            )

        self._fleet_pub = self.create_publisher(
            String, '/hospital/fleet_status', 10
        )
        self._assign_pub = self.create_publisher(
            String, '/hospital/assignments', 10
        )

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._assign_step)
        self.create_timer(5.0, self._publish_fleet)

        self.get_logger().info(
            f'Fleet Coordinator ready. '
            f'Managing: {self.ROBOTS}'
        )

    # ─────────────── Callbacks ───────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, robot_id: str):
        with self._lock:
            r = self._robots[robot_id]
            r.x = msg.pose.pose.position.x
            r.y = msg.pose.pose.position.y
            r.last_seen = time.time()

    def _status_cb(self, msg: String, robot_id: str):
        try:
            d     = json.loads(msg.data)
            event = d.get('event', '')
            tid   = d.get('task_id', '')
            with self._lock:
                r = self._robots[robot_id]
                if event in ('DELIVERED', 'FAILED'):
                    r.available    = True
                    r.current_task = None
                    r.task_count  += 1
                    if event == 'DELIVERED':
                        r.ok_count += 1
                elif event == 'NAVIGATING':
                    r.available    = False
                    r.current_task = tid
        except Exception:
            pass

    def _request_cb(self, msg: String):
        try:
            d    = json.loads(msg.data)
            dest = d.get('destination', 'nurse_station')
            if dest not in HOSPITAL_LOCATIONS:
                self.get_logger().warn(f'Unknown destination: {dest}')
                return
            x, y, _ = HOSPITAL_LOCATIONS[dest]
            task = PendingTask(
                task_id  = d.get('task_id', f'G{time.time():.0f}'),
                dest     = dest,
                dest_x   = x,
                dest_y   = y,
                priority = {'STAT': 0, 'URGENT': 1, 'ROUTINE': 2}.get(
                              d.get('priority', 'ROUTINE').upper(), 2),
                payload  = d.get('payload', 'medication'),
            )
            with self._lock:
                self._pending.append(task)
            self.get_logger().info(f'Fleet: queued {task.task_id} -> {dest}')
        except Exception as e:
            self.get_logger().error(f'Request error: {e}')

    # ─────────────── Assignment ───────────────────────────────────────────────

    def _assign_step(self):
        now = time.time()
        with self._lock:
            tasks = [t for t in self._pending if t.task_id not in self._assigned]
            avail = [
                r for r in self._robots.values()
                if r.available and (now - r.last_seen) < self.ROBOT_TIMEOUT
            ]

        if not tasks or not avail:
            return

        n_t, n_r = len(tasks), len(avail)
        cost = np.zeros((n_t, n_r))
        for i, t in enumerate(tasks):
            for j, r in enumerate(avail):
                dist = math.sqrt((t.dest_x - r.x)**2 + (t.dest_y - r.y)**2)
                pw   = 1.0 + t.priority * 0.2
                lw   = 1.0 + r.task_count * 0.05
                rel  = 2.0 - r.success_rate
                cost[i, j] = dist * pw * lw * rel

        # Hungarian: handle n_tasks != n_robots
        if n_t > n_r:
            ri, ci = linear_sum_assignment(cost.T)
            pairs  = list(zip(ci, ri))  # (task_idx, robot_idx)
        else:
            ri, ci = linear_sum_assignment(cost)
            pairs  = list(zip(ri, ci))

        log_entries = []
        with self._lock:
            for ti, rj in pairs:
                task  = tasks[ti]
                robot = avail[rj]
                self._assigned.add(task.task_id)
                robot.available    = False
                robot.current_task = task.task_id
                log_entries.append({
                    'task_id':  task.task_id,
                    'robot':    robot.robot_id,
                    'dest':     task.dest,
                    'cost':     round(float(cost[ti, rj]), 2),
                })

        # Dispatch outside lock
        for entry in log_entries:
            self._dispatch(entry['task_id'], entry['robot'], entry['dest'])
            self.get_logger().info(
                f"Assigned {entry['task_id']} -> {entry['robot']} "
                f"({entry['dest']}) cost={entry['cost']}"
            )

        msg      = String()
        msg.data = json.dumps({'assignments': log_entries, 'ts': time.time()})
        self._assign_pub.publish(msg)

    def _dispatch(self, task_id: str, robot_id: str, destination: str):
        # Find the original task
        with self._lock:
            task = next((t for t in self._pending if t.task_id == task_id), None)

        if not task:
            return

        payload  = {
            'task_id':     task.task_id,
            'origin':      'coordinator',
            'destination': task.dest,
            'payload':     task.payload,
            'priority':    ['STAT', 'URGENT', 'ROUTINE'][task.priority],
        }
        msg      = String()
        msg.data = json.dumps(payload)
        if robot_id in self._dispatch_pubs:
            self._dispatch_pubs[robot_id].publish(msg)

    def _publish_fleet(self):
        now = time.time()
        with self._lock:
            states = [
                {
                    'id':         r.robot_id,
                    'pos':        [round(r.x, 2), round(r.y, 2)],
                    'available':  r.available,
                    'task':       r.current_task,
                    'ok_rate':    round(r.success_rate, 2),
                    'online':     (now - r.last_seen) < self.ROBOT_TIMEOUT,
                }
                for r in self._robots.values()
            ]
            pending = len([t for t in self._pending if t.task_id not in self._assigned])

        msg      = String()
        msg.data = json.dumps({
            'robots':  states,
            'pending': pending,
            'total':   len(self._assigned),
            'ts':      now,
        })
        self._fleet_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
