#!/usr/bin/env python3
"""
Hospital Delivery Task Manager
================================
Manages delivery tasks for hospital robot fleet.

Features:
  - Priority-based task queue (STAT, URGENT, ROUTINE)
  - Nav2 action client integration
  - Task status tracking and logging
  - REST-like service interface for task submission
  - Multi-robot task routing (delegates to coordinator)

ROS 2 Jazzy | Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from hospital_mission.msg import DeliveryTask, TaskStatus

import json
import time
import threading
import heapq
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Optional


# ===================== CONSTANTS ==========================

class Priority(IntEnum):
    STAT    = 0   # Highest: emergency meds, blood products
    URGENT  = 1   # High: time-sensitive but not critical
    ROUTINE = 2   # Standard: lab samples, regular meds

class TaskState(str):
    PENDING    = "PENDING"
    NAVIGATING = "NAVIGATING"
    DELIVERED  = "DELIVERED"
    FAILED     = "FAILED"
    CANCELLED  = "CANCELLED"


# ===================== HOSPITAL LOCATIONS =================

HOSPITAL_LOCATIONS = {
    # Delivery destinations (x, y, yaw)
    "pharmacy":       ( 20.0,  -7.0, 0.0),
    "icu":            ( 18.0,   9.0, 0.0),
    "patient_room_1": (-19.5,   9.5, -1.57),
    "patient_room_2": (-14.5,   9.5, -1.57),
    "patient_room_3": ( -9.5,   9.5, -1.57),
    "patient_room_4": ( -4.5,   9.5, -1.57),
    "nurse_station_1":(  0.0,   0.5,  1.57),
    "nurse_station_2":( 15.0,   0.5,  1.57),
    "nurse_station_3":(-20.0,   0.5,  1.57),
    "lab":            ( 20.0,  -12.0, 0.0),
    "or_1":           ( -4.5,  10.5, -1.57),
    "emergency":      (-25.0,   0.0,  1.57),
    "home":           (  0.0,   0.0,  0.0),
}


# ===================== DATA CLASSES =======================

@dataclass(order=True)
class DeliveryTaskItem:
    priority:    int
    task_id:     str = field(compare=False)
    origin:      str = field(compare=False)
    destination: str = field(compare=False)
    payload:     str = field(compare=False)
    robot_id:    str = field(compare=False)
    state:       str = field(compare=False, default=TaskState.PENDING)
    created_at:  float = field(compare=False, default_factory=time.time)
    completed_at: Optional[float] = field(compare=False, default=None)


# ===================== TASK MANAGER NODE ==================

class HospitalTaskManager(Node):
    """
    Central task manager for hospital delivery robot system.
    Maintains priority queue, sends Nav2 goals, tracks delivery states.
    """

    def __init__(self):
        super().__init__('hospital_task_manager')

        # Parameters
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('nav2_action_server', 'navigate_to_pose')
        self.declare_parameter('max_concurrent_tasks', 1)
        self.declare_parameter('goal_tolerance', 0.3)

        self._ns       = self.get_parameter('robot_namespace').value
        self._robot_id = self.get_parameter('robot_id').value
        self._max_concurrent = self.get_parameter('max_concurrent_tasks').value

        # Task queue (min-heap by priority)
        self._task_queue:  list[DeliveryTaskItem] = []
        self._task_lock    = threading.Lock()
        self._active_tasks: dict[str, DeliveryTaskItem] = {}
        self._completed_tasks: list[DeliveryTaskItem] = []
        self._task_counter = 0

        # Nav2 action client
        ns_prefix = f'/{self._ns}' if self._ns else ''
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            f'{ns_prefix}/navigate_to_pose'
        )

        # Publishers
        self._status_pub = self.create_publisher(
            String,
            f'{ns_prefix}/delivery_status',
            10
        )

        self._task_pub = self.create_publisher(
            String,
            f'{ns_prefix}/task_log',
            10
        )

        # Subscribers
        self._task_sub = self.create_subscription(
            String,
            f'{ns_prefix}/delivery_request',
            self._task_request_callback,
            10
        )

        self._cancel_sub = self.create_subscription(
            String,
            f'{ns_prefix}/cancel_task',
            self._cancel_callback,
            10
        )

        # Processing timer
        self._process_timer = self.create_timer(1.0, self._process_queue)

        # Status timer
        self._status_timer = self.create_timer(5.0, self._publish_status)

        self.get_logger().info(
            f'[{self._robot_id}] Hospital Task Manager started. '
            f'Nav2 server: navigate_to_pose'
        )

    # ==================== TASK SUBMISSION =================

    def submit_task(self,
                    origin: str,
                    destination: str,
                    payload: str = "medication",
                    priority: Priority = Priority.ROUTINE) -> str:
        """Submit a new delivery task. Returns task ID."""
        self._task_counter += 1
        task_id = f"TASK_{self._robot_id}_{self._task_counter:04d}"

        task = DeliveryTaskItem(
            priority    = priority.value,
            task_id     = task_id,
            origin      = origin,
            destination = destination,
            payload     = payload,
            robot_id    = self._robot_id,
        )

        with self._task_lock:
            heapq.heappush(self._task_queue, task)

        self.get_logger().info(
            f'Task submitted: {task_id} | '
            f'{origin} -> {destination} | '
            f'Priority: {Priority(priority).name}'
        )

        self._publish_task_log(task_id, "SUBMITTED", destination)
        return task_id

    def _task_request_callback(self, msg: String):
        """Handle incoming task requests via ROS topic."""
        try:
            data = json.loads(msg.data)
            priority_str = data.get('priority', 'ROUTINE').upper()
            priority = getattr(Priority, priority_str, Priority.ROUTINE)

            task_id = self.submit_task(
                origin      = data.get('origin', 'pharmacy'),
                destination = data.get('destination', 'nurse_station_1'),
                payload     = data.get('payload', 'medication'),
                priority    = priority
            )

            self.get_logger().info(f'Task {task_id} queued from topic request')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Invalid task request: {e}')

    def _cancel_callback(self, msg: String):
        """Cancel a specific task by ID."""
        task_id = msg.data.strip()
        with self._task_lock:
            if task_id in self._active_tasks:
                self._active_tasks[task_id].state = TaskState.CANCELLED
                self.get_logger().warn(f'Task {task_id} cancelled (active)')
            else:
                for item in self._task_queue:
                    if item.task_id == task_id:
                        item.state = TaskState.CANCELLED
                        self.get_logger().warn(f'Task {task_id} cancelled (queued)')

    # ==================== QUEUE PROCESSING ================

    def _process_queue(self):
        """Main processing loop: pop next task and send Nav2 goal."""
        with self._task_lock:
            # Check capacity
            if len(self._active_tasks) >= self._max_concurrent:
                return

            if not self._task_queue:
                return

            task = heapq.heappop(self._task_queue)

            # Skip cancelled tasks
            if task.state == TaskState.CANCELLED:
                return

        self._execute_task(task)

    def _execute_task(self, task: DeliveryTaskItem):
        """Send Nav2 goal for the given task."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            task.state = TaskState.FAILED
            return

        if task.destination not in HOSPITAL_LOCATIONS:
            self.get_logger().error(
                f'Unknown destination: {task.destination}'
            )
            task.state = TaskState.FAILED
            return

        x, y, yaw = HOSPITAL_LOCATIONS[task.destination]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose_stamped(x, y, yaw)
        goal_msg.behavior_tree = ""

        task.state = TaskState.NAVIGATING
        with self._task_lock:
            self._active_tasks[task.task_id] = task

        self.get_logger().info(
            f'Executing task {task.task_id}: '
            f'navigating to {task.destination} ({x:.1f}, {y:.1f})'
        )

        self._publish_task_log(task.task_id, "NAVIGATING", task.destination)

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb, t=task: self._nav_feedback(fb, t)
        )
        send_future.add_done_callback(
            lambda f, t=task: self._goal_response_callback(f, t)
        )

    def _goal_response_callback(self, future, task: DeliveryTaskItem):
        """Called when Nav2 accepts/rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for task {task.task_id}')
            task.state = TaskState.FAILED
            with self._task_lock:
                self._active_tasks.pop(task.task_id, None)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, t=task: self._nav_result_callback(f, t)
        )

    def _nav_feedback(self, feedback_msg, task: DeliveryTaskItem):
        """Log navigation progress."""
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        if int(dist * 10) % 5 == 0:  # Log every 0.5m interval
            self.get_logger().debug(
                f'Task {task.task_id}: {dist:.2f}m remaining'
            )

    def _nav_result_callback(self, future, task: DeliveryTaskItem):
        """Handle Nav2 result."""
        result = future.result()
        status = result.status

        with self._task_lock:
            self._active_tasks.pop(task.task_id, None)
            task.completed_at = time.time()

        if status == 4:  # SUCCEEDED
            task.state = TaskState.DELIVERED
            self.get_logger().info(
                f'Task {task.task_id} DELIVERED to {task.destination}'
            )
            self._publish_task_log(task.task_id, "DELIVERED", task.destination)
        else:
            task.state = TaskState.FAILED
            self.get_logger().warn(
                f'Task {task.task_id} FAILED (Nav2 status: {status})'
            )
            self._publish_task_log(task.task_id, "FAILED", task.destination)

        self._completed_tasks.append(task)

    # ==================== HELPERS =========================

    def _build_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Build a PoseStamped from x, y, yaw."""
        import math
        from geometry_msgs.msg import Quaternion

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        q_z = math.sin(yaw / 2.0)
        q_w = math.cos(yaw / 2.0)
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w

        return pose

    def _publish_task_log(self, task_id: str, event: str, location: str):
        """Publish task event to log topic."""
        log = {
            "task_id":   task_id,
            "event":     event,
            "location":  location,
            "robot_id":  self._robot_id,
            "timestamp": time.time()
        }
        msg = String()
        msg.data = json.dumps(log)
        self._task_pub.publish(msg)

    def _publish_status(self):
        """Publish current system status."""
        with self._task_lock:
            status = {
                "robot_id":       self._robot_id,
                "queued_tasks":   len(self._task_queue),
                "active_tasks":   len(self._active_tasks),
                "completed":      len(self._completed_tasks),
                "active_list":    [
                    {"id": t.task_id, "dest": t.destination}
                    for t in self._active_tasks.values()
                ],
                "timestamp":      time.time()
            }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


# ===================== MAIN ===============================

def main(args=None):
    rclpy.init(args=args)

    node = HospitalTaskManager()

    # Demo: submit sample tasks on startup
    node.get_logger().info('Submitting demo delivery tasks...')
    node.submit_task('pharmacy', 'patient_room_1', 'antibiotics', Priority.URGENT)
    node.submit_task('pharmacy', 'icu',            'iv_drip',     Priority.STAT)
    node.submit_task('lab',      'nurse_station_1', 'blood_sample', Priority.ROUTINE)
    node.submit_task('pharmacy', 'patient_room_3', 'painkillers', Priority.ROUTINE)
    node.submit_task('pharmacy', 'or_1',           'surgical_kit', Priority.URGENT)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
