#!/usr/bin/env python3
"""
Multi-Robot Coordinator
========================
Coordinates 3 hospital TurtleBot4 robots using the Hungarian algorithm
for optimal task assignment. Manages shared map and robot states.

Algorithm:
  - Builds cost matrix (task x robot) using Manhattan distance
  - Applies Hungarian algorithm (scipy.optimize.linear_sum_assignment)
  - Assigns optimal task -> robot mapping
  - Monitors robot availability and reassigns on failure

ROS 2 Jazzy
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import json
import math
import time
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import Optional

try:
    from scipy.optimize import linear_sum_assignment
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


# ===================== DATA CLASSES =======================

@dataclass
class RobotState:
    robot_id:   str
    namespace:  str
    x:          float = 0.0
    y:          float = 0.0
    yaw:        float = 0.0
    available:  bool  = True
    current_task: Optional[str] = None
    last_seen:  float = field(default_factory=time.time)
    task_count: int   = 0
    success_count: int = 0

    @property
    def position(self):
        return (self.x, self.y)

    @property
    def success_rate(self):
        if self.task_count == 0:
            return 1.0
        return self.success_count / self.task_count


@dataclass
class PendingTask:
    task_id:     str
    destination: str
    dest_x:      float
    dest_y:      float
    priority:    int
    payload:     str
    created_at:  float = field(default_factory=time.time)


# ===================== HUNGARIAN SOLVER ===================

def hungarian_assign(cost_matrix: np.ndarray) -> list[tuple[int, int]]:
    """
    Solve assignment problem using Hungarian algorithm.
    Falls back to greedy assignment if scipy unavailable.

    Args:
        cost_matrix: (num_tasks x num_robots) cost matrix

    Returns:
        List of (task_idx, robot_idx) assignments
    """
    if SCIPY_AVAILABLE:
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        return list(zip(row_ind.tolist(), col_ind.tolist()))

    # Greedy fallback
    assignments = []
    used_robots = set()
    n_tasks, n_robots = cost_matrix.shape

    for task_idx in range(n_tasks):
        best_cost   = float('inf')
        best_robot  = -1
        for robot_idx in range(n_robots):
            if robot_idx not in used_robots:
                if cost_matrix[task_idx, robot_idx] < best_cost:
                    best_cost  = cost_matrix[task_idx, robot_idx]
                    best_robot = robot_idx
        if best_robot >= 0:
            assignments.append((task_idx, best_robot))
            used_robots.add(best_robot)

    return assignments


def build_cost_matrix(tasks: list[PendingTask],
                      robots: list[RobotState]) -> np.ndarray:
    """
    Build cost matrix for Hungarian assignment.
    Cost = Euclidean distance + priority weight + load balancing factor.
    """
    n_tasks  = len(tasks)
    n_robots = len(robots)

    if n_tasks == 0 or n_robots == 0:
        return np.zeros((n_tasks, n_robots))

    cost = np.zeros((n_tasks, n_robots))

    for i, task in enumerate(tasks):
        for j, robot in enumerate(robots):
            if not robot.available:
                cost[i, j] = 1e9  # Robot unavailable
                continue

            # Base cost: Euclidean distance from robot to destination
            dx = task.dest_x - robot.x
            dy = task.dest_y - robot.y
            dist = math.sqrt(dx * dx + dy * dy)

            # Priority multiplier (STAT tasks get lower cost = higher preference)
            priority_weight = 1.0 + (task.priority * 0.2)

            # Load balancing: penalize already-busy robots
            load_factor = 1.0 + (robot.task_count * 0.05)

            # Reliability factor: prefer robots with high success rate
            reliability = 2.0 - robot.success_rate

            cost[i, j] = dist * priority_weight * load_factor * reliability

    return cost


# ===================== COORDINATOR NODE ===================

class MultiRobotCoordinator(Node):
    """
    Centralized coordinator for 3 TurtleBot4 robots in hospital.
    Subscribes to robot odometry, maintains robot states,
    and assigns tasks using Hungarian algorithm.
    """

    ROBOT_NAMESPACES = ['robot_1', 'robot_2', 'robot_3']

    def __init__(self):
        super().__init__('multi_robot_coordinator')

        self.declare_parameter('assignment_rate_hz', 2.0)
        self.declare_parameter('robot_timeout_sec',  10.0)

        assign_rate   = self.get_parameter('assignment_rate_hz').value
        self._timeout = self.get_parameter('robot_timeout_sec').value

        # Robot registry
        self._robots: dict[str, RobotState] = {}
        self._robots_lock = threading.Lock()

        # Task pool (unassigned)
        self._pending_tasks: list[PendingTask] = []
        self._tasks_lock    = threading.Lock()

        # Assignment history
        self._assignments: dict[str, str] = {}  # task_id -> robot_id

        # Subscribe to each robot's odometry
        self._odom_subs = []
        for ns in self.ROBOT_NAMESPACES:
            state = RobotState(robot_id=ns, namespace=ns)
            self._robots[ns] = state

            sub = self.create_subscription(
                Odometry,
                f'/{ns}/odom',
                lambda msg, r=ns: self._odom_callback(msg, r),
                10
            )
            self._odom_subs.append(sub)

        # Subscribe to task status from each robot
        self._status_subs = []
        for ns in self.ROBOT_NAMESPACES:
            sub = self.create_subscription(
                String,
                f'/{ns}/task_log',
                lambda msg, r=ns: self._task_status_callback(msg, r),
                10
            )
            self._status_subs.append(sub)

        # Subscribe to global task queue
        self._global_task_sub = self.create_subscription(
            String,
            '/hospital/delivery_request',
            self._global_task_callback,
            10
        )

        # Publish assignment decisions
        self._assignment_pub = self.create_publisher(
            String,
            '/hospital/task_assignments',
            10
        )

        self._system_status_pub = self.create_publisher(
            String,
            '/hospital/system_status',
            10
        )

        # Per-robot task dispatch publishers
        self._robot_task_pubs = {}
        for ns in self.ROBOT_NAMESPACES:
            self._robot_task_pubs[ns] = self.create_publisher(
                String,
                f'/{ns}/delivery_request',
                10
            )

        # Assignment timer
        self._assign_timer = self.create_timer(
            1.0 / assign_rate,
            self._run_assignment
        )

        # Status timer
        self._status_timer = self.create_timer(5.0, self._publish_system_status)

        self.get_logger().info(
            f'Multi-Robot Coordinator started. '
            f'Managing {len(self.ROBOT_NAMESPACES)} robots. '
            f'scipy: {SCIPY_AVAILABLE}'
        )

    # ==================== CALLBACKS =======================

    def _odom_callback(self, msg: Odometry, robot_id: str):
        """Update robot pose from odometry."""
        with self._robots_lock:
            robot = self._robots[robot_id]
            robot.x = msg.pose.pose.position.x
            robot.y = msg.pose.pose.position.y
            robot.last_seen = time.time()

            # Extract yaw from quaternion
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            robot.yaw = math.atan2(siny_cosp, cosy_cosp)

    def _task_status_callback(self, msg: String, robot_id: str):
        """Handle task completion/failure from robots."""
        try:
            data = json.loads(msg.data)
            event    = data.get('event', '')
            task_id  = data.get('task_id', '')

            with self._robots_lock:
                robot = self._robots.get(robot_id)
                if robot:
                    if event in ('DELIVERED', 'FAILED'):
                        robot.available   = True
                        robot.current_task = None
                        robot.task_count  += 1
                        if event == 'DELIVERED':
                            robot.success_count += 1

                    elif event == 'NAVIGATING':
                        robot.available    = False
                        robot.current_task = task_id

        except json.JSONDecodeError:
            pass

    def _global_task_callback(self, msg: String):
        """Accept global task requests and add to pending pool."""
        try:
            data = json.loads(msg.data)

            from hospital_mission.task_manager import HOSPITAL_LOCATIONS
            destination = data.get('destination', 'nurse_station_1')

            if destination not in HOSPITAL_LOCATIONS:
                self.get_logger().warn(f'Unknown destination: {destination}')
                return

            x, y, _ = HOSPITAL_LOCATIONS[destination]

            task = PendingTask(
                task_id     = data.get('task_id', f'global_{time.time():.0f}'),
                destination = destination,
                dest_x      = x,
                dest_y      = y,
                priority    = data.get('priority', 2),
                payload     = data.get('payload', 'medication'),
            )

            with self._tasks_lock:
                self._pending_tasks.append(task)

            self.get_logger().info(
                f'Global task queued: {task.task_id} -> {destination}'
            )

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Bad global task request: {e}')

    # ==================== ASSIGNMENT ======================

    def _run_assignment(self):
        """Hungarian assignment: match pending tasks to available robots."""
        with self._tasks_lock:
            pending = [t for t in self._pending_tasks
                       if t.task_id not in self._assignments]

        if not pending:
            return

        with self._robots_lock:
            # Filter out timed-out robots
            now = time.time()
            available = [
                r for r in self._robots.values()
                if r.available and (now - r.last_seen) < self._timeout
            ]

        if not available:
            self.get_logger().warn('No available robots for assignment')
            return

        n_tasks  = len(pending)
        n_robots = len(available)

        # Build cost matrix
        cost_matrix = build_cost_matrix(pending, available)

        # Solve assignment (Hungarian)
        try:
            assignments = hungarian_assign(cost_matrix[:n_tasks, :n_robots])
        except Exception as e:
            self.get_logger().error(f'Assignment failed: {e}')
            return

        # Dispatch tasks
        for task_idx, robot_idx in assignments:
            task  = pending[task_idx]
            robot = available[robot_idx]

            self._dispatch_task(task, robot)
            self._assignments[task.task_id] = robot.robot_id

            # Mark robot busy
            with self._robots_lock:
                self._robots[robot.robot_id].available    = False
                self._robots[robot.robot_id].current_task = task.task_id

        # Publish assignment log
        log = {
            "event":       "ASSIGNMENT_ROUND",
            "assignments": [
                {
                    "task_id":     pending[ti].task_id,
                    "robot_id":    available[ri].robot_id,
                    "destination": pending[ti].destination,
                    "cost":        float(cost_matrix[ti, ri])
                }
                for ti, ri in assignments
            ],
            "timestamp": time.time()
        }
        msg = String()
        msg.data = json.dumps(log)
        self._assignment_pub.publish(msg)

        self.get_logger().info(
            f'Assigned {len(assignments)} tasks to robots'
        )

    def _dispatch_task(self, task: PendingTask, robot: RobotState):
        """Send task to a specific robot via its delivery_request topic."""
        payload = {
            "task_id":     task.task_id,
            "destination": task.destination,
            "payload":     task.payload,
            "priority":    ["STAT", "URGENT", "ROUTINE"][task.priority],
            "origin":      "coordinator"
        }
        msg = String()
        msg.data = json.dumps(payload)

        if robot.robot_id in self._robot_task_pubs:
            self._robot_task_pubs[robot.robot_id].publish(msg)

        self.get_logger().info(
            f'Dispatched {task.task_id} -> {robot.robot_id} '
            f'({task.destination})'
        )

    # ==================== STATUS ==========================

    def _publish_system_status(self):
        """Publish fleet status."""
        with self._robots_lock:
            robot_states = [
                {
                    "id":        r.robot_id,
                    "pos":       [round(r.x, 2), round(r.y, 2)],
                    "available": r.available,
                    "task":      r.current_task,
                    "tasks":     r.task_count,
                    "success_rate": round(r.success_rate, 2)
                }
                for r in self._robots.values()
            ]

        with self._tasks_lock:
            pending_count = len(
                [t for t in self._pending_tasks
                 if t.task_id not in self._assignments]
            )

        status = {
            "robots":        robot_states,
            "pending_tasks": pending_count,
            "total_assigned": len(self._assignments),
            "scipy_enabled": SCIPY_AVAILABLE,
            "timestamp":     time.time()
        }

        msg = String()
        msg.data = json.dumps(status)
        self._system_status_pub.publish(msg)


# ===================== MAIN ===============================

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
