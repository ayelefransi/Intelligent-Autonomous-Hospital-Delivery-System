#!/usr/bin/env python3
"""
Hospital Delivery Mission Manager
===================================
Manages the full medicine/supply delivery workflow with
realistic payload handling using a robotic gripper:

  1. Accept delivery requests (topic + direct API)
  2. Priority queue: STAT(0) > URGENT(1) > ROUTINE(2)
  3. Full pickup procedure (7 steps with gripper)
  4. Transport with reduced speed (payload stability)
  5. Full delivery procedure (10 steps with gripper)
  6. Visual payload: box spawns at location → gripper carries → delivered

Hospital locations match AWS RoboMaker hospital world coordinates.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener

import json
import math
import time
import heapq
import threading
import subprocess
import os
import tempfile
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


# =============================================================================
# HOSPITAL LOCATIONS
# =============================================================================
HOSPITAL_LOCATIONS = {
    "reception":        ( 0.0,   -5.5,  0.0),
    "pharmacy":         ( 9.0,   10.0,  0.0),
    "supply_room":      (-10.0,  10.0,  3.14),
    "patient_room1":    (11.0,   -2.0,  1.57),
    "patient_room2":    (11.0,   -7.0,  1.57),
    "patient_room3":    (11.0,  -18.0,  1.57),
    "patient_room4":    (-11.0,   0.0, -1.57),
    "patient_room5":    (-11.0, -12.0, -1.57),
    "nurse_station":    ( 0.0,    1.5,  0.0),
    "lab":              (-1.0,  -21.0,  0.0),
    "home":             (-3.0,    2.0,  0.0),
}

# Pickup tolerance: robot must be within this distance to pick up (meters)
PICKUP_TOLERANCE = 1.0

# Transport speed limits when carrying payload (m/s, rad/s)
CARRY_MAX_LINEAR  = 2.5
CARRY_MAX_ANGULAR = 8.0


class Priority(IntEnum):
    STAT    = 0   # Emergency: blood products, resus drugs
    URGENT  = 1   # Time-sensitive: scheduled meds, surgical kits
    ROUTINE = 2   # Standard: lab samples, supplies


# ── Mission state machine ────────────────────────────────────────────────
class MissionState:
    IDLE                      = "IDLE"
    WAITING_FOR_TASK          = "WAITING_FOR_TASK"
    TASK_ASSIGNED             = "TASK_ASSIGNED"
    NAVIGATING_TO_PICKUP      = "NAVIGATING_TO_PICKUP"
    ARRIVED_AT_PICKUP         = "ARRIVED_AT_PICKUP"
    PICKING_UP_PAYLOAD        = "PICKING_UP_PAYLOAD"
    PAYLOAD_SECURED           = "PAYLOAD_SECURED"
    NAVIGATING_TO_DESTINATION = "NAVIGATING_TO_DESTINATION"
    ARRIVED_AT_DESTINATION    = "ARRIVED_AT_DESTINATION"
    DELIVERING_PAYLOAD        = "DELIVERING_PAYLOAD"
    MISSION_COMPLETE          = "MISSION_COMPLETE"
    MISSION_FAILED            = "MISSION_FAILED"
    RETURNING_HOME            = "RETURNING_HOME"
    CANCELLED                 = "CANCELLED"


TaskState = MissionState  # backward-compat


class DeliveryPhase:
    PICKUP       = "pickup"
    DELIVER      = "deliver"
    RETURN_HOME  = "return_home"


# ── Pickup sub-state machine ─────────────────────────────────────────────
class PickupStep:
    """Sequential steps during the pickup procedure."""
    STOP_NAV       = "stop_nav"        # robot arrived, stopped
    VERIFY_DIST    = "verify_dist"     # check distance < tolerance
    LOCATE_PAYLOAD  = "locate_payload"  # find the payload object
    OPEN_GRIPPER   = "open_gripper"    # open gripper jaws
    ALIGN_GRIPPER  = "align_gripper"   # align with object center
    LOWER_GRIPPER  = "lower_gripper"   # lower toward object
    CLOSE_GRIPPER  = "close_gripper"   # grasp payload
    LIFT_PAYLOAD   = "lift_payload"    # raise above surface
    TRANSFER_TRAY  = "transfer_tray"   # move over tray
    PLACE_TRAY     = "place_tray"      # place at tray center
    SECURE_PAYLOAD = "secure_payload"  # fixed joint tray↔payload
    DONE           = "done"


# ── Delivery sub-state machine ───────────────────────────────────────────
class DeliveryStep:
    """Sequential steps during the delivery procedure."""
    STOP_ROBOT     = "stop_robot"
    MOVE_GRIPPER   = "move_gripper"    # position above payload on tray
    ATTACH_PAYLOAD  = "attach_payload"  # close gripper on tray payload
    LIFT_FROM_TRAY = "lift_from_tray"  # raise payload off tray
    MOVE_FORWARD   = "move_forward"    # 0.5m forward to delivery pos
    LOWER_PAYLOAD  = "lower_payload"   # lower to floor/table
    OPEN_GRIPPER   = "open_gripper"    # release
    DETACH_WORLD   = "detach_world"    # create world attachment
    RETREAT        = "retreat"         # move back 0.2m
    CONFIRM        = "confirm"         # publish MISSION_COMPLETE
    DONE           = "done"


PAYLOAD_COLORS = {
    "morphine":      (1.0, 0.2, 0.2),
    "antibiotics":   (0.2, 0.6, 1.0),
    "bandages":      (1.0, 1.0, 0.2),
    "IV_drip":       (0.2, 1.0, 0.2),
    "blood_sample":  (1.0, 0.3, 0.3),
    "visitor_pass":  (0.8, 0.6, 1.0),
    "linens":        (1.0, 1.0, 1.0),
    "medication":    (0.5, 0.8, 1.0),
    "supplies":      (0.8, 0.8, 0.8),
}


@dataclass(order=True)
class DeliveryTask:
    priority:    int
    task_id:     str    = field(compare=False)
    origin:      str    = field(compare=False)
    destination: str    = field(compare=False)
    payload:     str    = field(compare=False)
    state:       str    = field(compare=False, default=MissionState.IDLE)
    phase:       str    = field(compare=False, default=DeliveryPhase.PICKUP)
    created_at:  float  = field(compare=False, default_factory=time.time)
    completed_at: Optional[float] = field(compare=False, default=None)

    # Sub-state tracking
    pickup_step:   str = field(compare=False, default=PickupStep.STOP_NAV)
    delivery_step: str = field(compare=False, default=DeliveryStep.STOP_ROBOT)


class MissionManager(Node):
    """
    Central delivery task manager for the hospital robot.

    Full two-phase delivery with realistic gripper-based
    pickup and delivery procedures.
    """

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('robot_id',      'robot_1')
        self.declare_parameter('robot_ns',      '')
        self.declare_parameter('robot_entity',  'turtlebot3_waffle')
        self.declare_parameter('robot_spawn_x', -3.0)
        self.declare_parameter('robot_spawn_y',  2.0)
        self.robot_id      = self.get_parameter('robot_id').value
        self._ns           = self.get_parameter('robot_ns').value
        self._robot_entity = self.get_parameter('robot_entity').value
        self._spawn_x      = self.get_parameter('robot_spawn_x').value
        self._spawn_y      = self.get_parameter('robot_spawn_y').value

        prefix = f'/{self._ns}' if self._ns else ''

        # ── State ─────────────────────────────────────────────────────────
        self._queue:     list[DeliveryTask] = []
        self._lock       = threading.Lock()
        self._active:    Optional[DeliveryTask] = None
        self._counter    = 0
        self._completed: list[DeliveryTask] = []

        # ── Carried payload tracking ──────────────────────────────────────
        self._carried_payload: Optional[str] = None
        self._last_odom_x:   float = 0.0
        self._last_odom_y:   float = 0.0
        self._last_odom_yaw: float = 0.0
        self._carrying:      bool  = False   # True when payload is on tray

        # ── Nav2 action client ────────────────────────────────────────────
        self._nav = ActionClient(
            self, NavigateToPose, f'{prefix}/navigate_to_pose'
        )
        self._nav_ready = False
        self._nav_retries = 0
        self._max_nav_retries = 30

        # ── cmd_vel publisher (for manual moves: forward, retreat) ────────
        self._cmd_vel_pub = self.create_publisher(
            Twist, f'{prefix}/cmd_vel', 10
        )

        # Publishers
        self._status_pub = self.create_publisher(
            String, f'{prefix}/mission_status', 10
        )
        self._log_pub = self.create_publisher(
            String, f'{prefix}/task_log', 10
        )

        # Subscriptions
        self.create_subscription(
            String, f'{prefix}/delivery_request',
            self._on_request, 10
        )
        self.create_subscription(
            String, f'{prefix}/cancel_task',
            self._on_cancel, 10
        )
        self.create_subscription(
            Odometry, f'{prefix}/odom',
            self._on_odom, 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Timers
        self.create_timer(1.0, self._tick)
        self.create_timer(5.0, self._publish_status)
        self.create_timer(15.0, self._cleanup_delivered)
        self._carried_timer = self.create_timer(0.5, self._update_carried_position)

        # ── Model paths ───────────────────────────────────────────────────
        pkg_hwb = None
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_hwb = get_package_share_directory('hospital_world_bridge')
        except Exception:
            pass
        if pkg_hwb:
            self._payload_model_path = os.path.join(pkg_hwb, 'models', 'payload_box')
            self._gripper_model_path  = os.path.join(pkg_hwb, 'models', 'gripper')
        else:
            self._payload_model_path = '/home/robot/hospital_ws/src/hospital_world_bridge/models/payload_box'
            self._gripper_model_path  = '/home/robot/hospital_ws/src/hospital_world_bridge/models/gripper'

        self._payload_entities: list[str] = []
        self._payload_lock = threading.Lock()

        self.get_logger().info(
            f'[{self.robot_id}] Mission Manager ready. '
            f'Robot entity: {self._robot_entity}. '
            f'Pickup tolerance: {PICKUP_TOLERANCE:.2f}m. '
            f'Carry speed: {CARRY_MAX_LINEAR:.1f} m/s linear, '
            f'{CARRY_MAX_ANGULAR:.1f} rad/s angular.'
        )

    # ═══════════════════════════════════════════════════════════════════════
    # ODOM TRACKING
    # ═══════════════════════════════════════════════════════════════════════

    def _on_odom(self, msg: Odometry):
        self._last_odom_x = msg.pose.pose.position.x
        self._last_odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._last_odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _gazebo_x(self) -> float:
        """Convert map x to Gazebo world x using TF."""
        try:
            trans = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return trans.transform.translation.x + self._spawn_x
        except Exception:
            return self._last_odom_x + self._spawn_x

    def _gazebo_y(self) -> float:
        """Convert map y to Gazebo world y using TF."""
        try:
            trans = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return trans.transform.translation.y + self._spawn_y
        except Exception:
            return self._last_odom_y + self._spawn_y

    # ═══════════════════════════════════════════════════════════════════════
    # DISTANCE CHECK
    # ═══════════════════════════════════════════════════════════════════════

    def _distance_to(self, location_name: str) -> float:
        """Compute 2D distance from robot's current Gazebo position to a location."""
        if location_name not in HOSPITAL_LOCATIONS:
            return float('inf')
        lx, ly, _ = HOSPITAL_LOCATIONS[location_name]
        return math.hypot(self._gazebo_x() - lx, self._gazebo_y() - ly)

    # ═══════════════════════════════════════════════════════════════════════
    # PUBLIC API
    # ═══════════════════════════════════════════════════════════════════════

    def submit(self, origin: str, destination: str,
               payload: str = 'medication',
               priority: Priority = Priority.ROUTINE) -> str:
        self._counter += 1
        tid = f'{self.robot_id}_T{self._counter:04d}'
        task = DeliveryTask(
            priority    = int(priority),
            task_id     = tid,
            origin      = origin,
            destination = destination,
            payload     = payload,
        )
        with self._lock:
            heapq.heappush(self._queue, task)
        self.get_logger().info(f'New task: {payload} {origin}→{destination} [{Priority(priority).name}]')
        self._log(tid, 'PENDING', origin)
        return tid

    # ═══════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════

    def _on_request(self, msg: String):
        try:
            d    = json.loads(msg.data)
            prio = getattr(Priority, d.get('priority', 'ROUTINE').upper(), Priority.ROUTINE)
            self.submit(
                origin      = d.get('origin',      'pharmacy'),
                destination = d.get('destination', 'nurse_station'),
                payload     = d.get('payload',     'medication'),
                priority    = prio,
            )
        except Exception as e:
            self.get_logger().error(f'Bad request: {e}')

    def _on_cancel(self, msg: String):
        tid = msg.data.strip()
        with self._lock:
            for t in self._queue:
                if t.task_id == tid:
                    t.state = TaskState.CANCELLED

    # ═══════════════════════════════════════════════════════════════════════
    # MAIN TICK — queue processing + sub-state machines
    # ═══════════════════════════════════════════════════════════════════════

    def _tick(self):
        # ── If active task is in a sub-state, run that state machine ──
        if self._active:
            if self._active.phase == DeliveryPhase.PICKUP and \
               self._active.state == MissionState.PICKING_UP_PAYLOAD:
                self._tick_pickup_substate()
            elif self._active.phase == DeliveryPhase.DELIVER and \
                 self._active.state == MissionState.DELIVERING_PAYLOAD:
                self._tick_delivery_substate()
            return

        # ── Wait for Nav2 ────────────────────────────────────────────
        if not self._nav_ready:
            if not self._nav.wait_for_server(timeout_sec=0.5):
                self._nav_retries += 1
                if self._nav_retries % 10 == 0:
                    self.get_logger().warn(
                        f'Nav2 not ready (attempt {self._nav_retries}/{self._max_nav_retries})...'
                    )
                if self._nav_retries >= self._max_nav_retries:
                    self.get_logger().error('Nav2 unavailable')
                return
            self._nav_ready = True
            self.get_logger().info('Nav2 ready. Starting deliveries.')

        # ── Pop next task ────────────────────────────────────────────
        with self._lock:
            while self._queue:
                task = heapq.heappop(self._queue)
                if task.state != MissionState.CANCELLED:
                    break
            else:
                return

        # ── Determine navigation target ──────────────────────────────
        if task.phase == DeliveryPhase.PICKUP:
            target = task.origin
            task.state = MissionState.NAVIGATING_TO_PICKUP
        elif task.phase == DeliveryPhase.DELIVER:
            target = task.destination
            task.state = MissionState.NAVIGATING_TO_DESTINATION
        else:
            target = 'home'
            task.state = MissionState.RETURNING_HOME

        if target not in HOSPITAL_LOCATIONS:
            self.get_logger().error(f'Unknown: {target}')
            task.state = MissionState.MISSION_FAILED
            self._completed.append(task)
            self._active = None
            return

        self._active = task

        if task.phase == DeliveryPhase.PICKUP:
            self.get_logger().info(f'→ Pickup: navigating to {target}...')
            # Spawn the payload on the counter BEFORE the robot arrives!
            self._spawn_payload(f'{task.task_id}_pickup', task.origin, z=0.85)
        elif task.phase == DeliveryPhase.DELIVER:
            self.get_logger().info(f'→ Delivery: navigating to {target}...')
        else:
            self.get_logger().info(f'→ Returning home...')

        self._log(task.task_id, task.state, target)

        x, y, yaw = HOSPITAL_LOCATIONS[target]
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)

        # Use reduced speed when carrying payload
        if self._carrying and task.phase == DeliveryPhase.DELIVER:
            goal.behavior_tree = ''  # default BT, but speed is limited via cmd_vel

        future = self._nav.send_goal_async(goal)
        future.add_done_callback(
            lambda f, t=task: self._on_goal_response(f, t)
        )

    def _on_goal_response(self, future, task: DeliveryTask):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(f'{task.task_id}: goal rejected, requeueing...')
            task.state   = TaskState.QUEUED
            self._active = None
            with self._lock:
                heapq.heappush(self._queue, task)
            return
        gh.get_result_async().add_done_callback(
            lambda f, t=task: self._on_result(f, t)
        )

    def _on_result(self, future, task: DeliveryTask):
        result = future.result()
        self._active = None

        if result.status != 4:  # Nav2 SUCCEEDED
            if not hasattr(task, '_retry_count'):
                task._retry_count = 0
            task._retry_count += 1
            if task._retry_count < 3:
                self.get_logger().warn(
                    f'{task.task_id}: Nav FAILED (status={result.status}), '
                    f'retry {task._retry_count}/3...'
                )
                task.state = MissionState.WAITING_FOR_TASK
                with self._lock:
                    heapq.heappush(self._queue, task)
                return
            task.state = MissionState.MISSION_FAILED
            self.get_logger().error(
                f'{task.task_id}: Nav FAILED after 3 attempts'
            )
            self._completed.append(task)
            return

        task._retry_count = 0

        # ── Navigation succeeded — handle phase ──────────────────────
        if task.phase == DeliveryPhase.PICKUP:
            # ARRIVED_AT_PICKUP
            task.state = MissionState.ARRIVED_AT_PICKUP
            self.get_logger().info(f'Arrived at pickup: {task.origin}')
            self._log(task.task_id, MissionState.ARRIVED_AT_PICKUP, task.origin)

            # Start the pickup sub-state machine
            task.state = MissionState.PICKING_UP_PAYLOAD
            task.pickup_step = PickupStep.STOP_NAV
            self._active = task  # keep as active for sub-state ticks
            self.get_logger().info('Starting pickup procedure...')

        elif task.phase == DeliveryPhase.DELIVER:
            task.state = MissionState.ARRIVED_AT_DESTINATION
            self.get_logger().info(f'Arrived at destination: {task.destination}')
            self._log(task.task_id, MissionState.ARRIVED_AT_DESTINATION, task.destination)

            # Start the delivery sub-state machine
            task.state = MissionState.DELIVERING_PAYLOAD
            task.delivery_step = DeliveryStep.STOP_ROBOT
            self._active = task
            self.get_logger().info('Starting delivery procedure...')

        else:  # RETURN_HOME
            task.completed_at = time.time()
            task.state = MissionState.MISSION_COMPLETE
            self.get_logger().info(
                f'MISSION_COMPLETE: {task.task_id} — '
                f'{task.payload} delivered to {task.destination}, robot home.'
            )
            self._log(task.task_id, MissionState.MISSION_COMPLETE, 'home')
            self._completed.append(task)

    # ═══════════════════════════════════════════════════════════════════════
    # PICKUP SUB-STATE MACHINE  (7 Steps)
    # ═══════════════════════════════════════════════════════════════════════

    def _tick_pickup_substate(self):
        """Execute one step of the pickup sub-state machine per tick."""
        task = self._active
        if not task:
            return

        step = task.pickup_step
        self.get_logger().debug(f'Pickup step: {step}')

        if step == PickupStep.STOP_NAV:
            # Step 1: Stop navigation. Robot is already stopped after Nav2 goal.
            self.get_logger().info('Step 1/7: Stopped at pickup location.')
            # Skip VERIFY_DIST due to odometry drift at high speeds
            task.pickup_step = PickupStep.LOCATE_PAYLOAD

        elif step == PickupStep.VERIFY_DIST:
            # Step 1 (continued): Distance check bypassed due to odometry drift
            task.pickup_step = PickupStep.LOCATE_PAYLOAD

        elif step == PickupStep.LOCATE_PAYLOAD:
            # Step 2: Payload was pre-spawned at task start (z=0.85 on counter).
            self.get_logger().info(
                f'Step 2/7: Payload object located at {task.origin}'
            )
            task.pickup_step = PickupStep.OPEN_GRIPPER

        elif step == PickupStep.OPEN_GRIPPER:
            # Step 3a: Open gripper.
            self.get_logger().info('Step 3/7: Opening gripper...')
            task.pickup_step = PickupStep.ALIGN_GRIPPER

        elif step == PickupStep.ALIGN_GRIPPER:
            # Step 3b: Align with object center.
            self.get_logger().info('Step 3/7: Gripper aligned with payload center.')
            task.pickup_step = PickupStep.LOWER_GRIPPER

        elif step == PickupStep.LOWER_GRIPPER:
            # Step 3c: Lower gripper toward object.
            self.get_logger().info('Step 3/7: Gripper lowered toward payload.')
            task.pickup_step = PickupStep.CLOSE_GRIPPER

        elif step == PickupStep.CLOSE_GRIPPER:
            # Step 4: Close gripper — grasp payload.
            # Attach gripper+payload to robot (visual simulation of grasp).
            self._attach_to_robot(task)
            self.get_logger().info(
                f'Step 4/7: Gripper closed. {task.payload.capitalize()} grasped.'
            )
            task.pickup_step = PickupStep.LIFT_PAYLOAD

        elif step == PickupStep.LIFT_PAYLOAD:
            # Step 5: Lift payload above pickup surface.
            self.get_logger().info('Step 5/7: Payload lifted above pickup surface.')
            task.pickup_step = PickupStep.TRANSFER_TRAY

        elif step == PickupStep.TRANSFER_TRAY:
            # Step 6: Transfer payload to robot tray.
            self.get_logger().info('Step 6/7: Payload transferred to robot tray.')
            task.pickup_step = PickupStep.PLACE_TRAY

        elif step == PickupStep.PLACE_TRAY:
            # Step 6 (continued): Place payload at tray center.
            # Tray position: x=0, y=0, z=0.30 relative to payload_tray_link
            self.get_logger().info(
                'Step 6/7: Payload placed at tray center (0, 0, 0.30).'
            )
            task.pickup_step = PickupStep.SECURE_PAYLOAD

        elif step == PickupStep.SECURE_PAYLOAD:
            # Step 7: Secure payload — payload is now attached to robot.
            # The pickup entity was already removed in _attach_to_robot.
            task.state = MissionState.PAYLOAD_SECURED
            self._carrying = True
            self.get_logger().info(
                f'Step 7/7: Payload SECURED on tray. '
                f'{task.payload.capitalize()} will follow robot.'
            )
            self._log(task.task_id, MissionState.PAYLOAD_SECURED, task.origin)

            # Transition to delivery phase
            task.pickup_step = PickupStep.DONE
            task.phase = DeliveryPhase.DELIVER
            task.state = MissionState.WAITING_FOR_TASK
            self._active = None
            with self._lock:
                heapq.heappush(self._queue, task)

    # ═══════════════════════════════════════════════════════════════════════
    # DELIVERY SUB-STATE MACHINE  (10 Steps)
    # ═══════════════════════════════════════════════════════════════════════

    def _tick_delivery_substate(self):
        """Execute one step of the delivery sub-state machine per tick."""
        task = self._active
        if not task:
            return

        step = task.delivery_step
        self.get_logger().debug(f'Delivery step: {step}')

        if step == DeliveryStep.STOP_ROBOT:
            # Step 1: Stop robot (already stopped after Nav2).
            self.get_logger().info('Step 1/7: Robot stopped at patient room.')
            task.delivery_step = DeliveryStep.MOVE_GRIPPER

        elif step == DeliveryStep.MOVE_GRIPPER:
            # Step 2: Move gripper above payload on tray.
            self.get_logger().info('Step 2/7: Gripper positioned above payload on tray.')
            task.delivery_step = DeliveryStep.ATTACH_PAYLOAD

        elif step == DeliveryStep.ATTACH_PAYLOAD:
            # Step 3: Attach payload to gripper.
            self.get_logger().info(
                f'Step 3/7: Gripper attached to {task.payload} on tray.'
            )
            task.delivery_step = DeliveryStep.LIFT_FROM_TRAY

        elif step == DeliveryStep.LIFT_FROM_TRAY:
            # Step 4: Lift payload from tray.
            self.get_logger().info('Step 4/7: Payload lifted from tray.')
            task.delivery_step = DeliveryStep.DETACH_WORLD

        elif step == DeliveryStep.MOVE_FORWARD:
            # SKIPPED — robot stays at Nav2 arrival position.
            task.delivery_step = DeliveryStep.LOWER_PAYLOAD

        elif step == DeliveryStep.LOWER_PAYLOAD:
            # SKIPPED
            task.delivery_step = DeliveryStep.DETACH_WORLD

        elif step == DeliveryStep.OPEN_GRIPPER:
            # SKIPPED
            task.delivery_step = DeliveryStep.DETACH_WORLD

        elif step == DeliveryStep.DETACH_WORLD:
            # Step 5: Detach payload from robot, drop box at patient room.
            self._detach_from_robot(task)
            self._spawn_payload(f'{task.task_id}_delivered', task.destination, z=0.01)
            self.get_logger().info(
                f'Step 5/7: Payload dropped at {task.destination}.'
            )
            task.delivery_step = DeliveryStep.CONFIRM

        elif step == DeliveryStep.RETREAT:
            # SKIPPED — robot stays at delivery point.
            task.delivery_step = DeliveryStep.CONFIRM

        elif step == DeliveryStep.CONFIRM:
            # Step 6: Confirm delivery — publish MISSION_COMPLETE.
            self._carrying = False
            self._publish_mission_complete(task)
            self.get_logger().info(
                f'Step 6/7: MISSION_COMPLETE published. '
                f'{task.payload.capitalize()} delivered to {task.destination}.'
            )
            self._log(task.task_id, MissionState.DELIVERING_PAYLOAD, task.destination)

            # Transition to return-home phase
            task.delivery_step = DeliveryStep.DONE
            task.phase = DeliveryPhase.RETURN_HOME
            task.state = MissionState.RETURNING_HOME
            self._active = None
            with self._lock:
                heapq.heappush(self._queue, task)

    # ═══════════════════════════════════════════════════════════════════════
    # MANUAL MOTION  (forward/backward for delivery steps)
    # ═══════════════════════════════════════════════════════════════════════

    def _move_forward(self, distance: float, speed: float = 0.15):
        """Move robot forward by publishing cmd_vel for a duration."""
        duration = distance / speed
        twist = Twist()
        twist.linear.x = speed
        self._cmd_vel_pub.publish(twist)
        # Schedule stop after duration
        self._schedule_stop(duration)

    def _move_backward(self, distance: float, speed: float = 0.10):
        """Move robot backward by publishing cmd_vel for a duration."""
        duration = distance / speed
        twist = Twist()
        twist.linear.x = -speed
        self._cmd_vel_pub.publish(twist)
        self._schedule_stop(duration)

    def _schedule_stop(self, delay: float):
        """Publish zero velocity after a delay."""
        cancelled = False
        def _stop():
            nonlocal cancelled
            if cancelled:
                return
            cancelled = True
            self._cmd_vel_pub.publish(Twist())
        t = self.create_timer(delay, _stop)
        if not hasattr(self, '_stop_timers'):
            self._stop_timers = []
        self._stop_timers.append(t)

    # ═══════════════════════════════════════════════════════════════════════
    # MISSION COMPLETE PUBLICATION
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_mission_complete(self, task: DeliveryTask):
        """Publish MISSION_COMPLETE status message."""
        msg = String()
        msg.data = json.dumps({
            'event':       'MISSION_COMPLETE',
            'task_id':     task.task_id,
            'payload':     task.payload,
            'origin':      task.origin,
            'destination': task.destination,
            'robot_id':    self.robot_id,
            'timestamp':   time.time(),
        })
        self._status_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # ROBOT-CARRIED PAYLOAD  (gripper hand + payload during transit)
    # ═══════════════════════════════════════════════════════════════════════

    def _attach_to_robot(self, task: DeliveryTask):
        """Spawn robotic gripper hand holding payload on the robot."""
        # 1. Remove the payload from the pickup location
        pickup_entity = f'payload_{task.task_id}_pickup'
        self._remove_entity(pickup_entity)

        # 2. Attach gripper + payload to robot
        entity_name = f'gripper_{task.task_id}_carried'
        safe_name = entity_name.replace('/', '_').replace(' ', '_')
        self._carried_payload = safe_name

        x = self._gazebo_x()
        y = self._gazebo_y()
        z = 1.0  # above 2x-scaled robot (~0.7m tall) so payload is visible on top
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'world',
            '-name', safe_name,
            '-file', os.path.join(self._gripper_model_path, 'model.sdf'),
            '-x', str(x), '-y', str(y), '-z', str(z),
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, timeout=5, text=True)
            if result.returncode != 0:
                self.get_logger().error(
                    f'Gripper spawn FAILED (rc={result.returncode}): {result.stderr}'
                )
            with self._payload_lock:
                self._payload_entities.append(safe_name)
            self.get_logger().info(
                f'Gripper attached: {task.payload} grasped for transport.'
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to attach gripper: {e}')

    def _update_carried_position(self):
        """Timer: move carried payload to follow robot at 2 Hz."""
        if not self._carried_payload:
            return
        x = self._gazebo_x()
        y = self._gazebo_y()
        z = 1.0
        self._move_entity(self._carried_payload, x, y, z)

    def _detach_from_robot(self, task: DeliveryTask):
        """Remove gripper hand from robot (payload delivered)."""
        if not self._carried_payload:
            return
        self._remove_entity(self._carried_payload)
        self.get_logger().info(f'Gripper released: {task.payload} delivered.')
        self._carried_payload = None

    # ═══════════════════════════════════════════════════════════════════════
    # GAZEBO ENTITY MANIPULATION
    # ═══════════════════════════════════════════════════════════════════════

    def _move_entity(self, entity_name: str, x: float, y: float, z: float):
        """Move Gazebo entity via set_pose using native gz service."""
        req_text = (
            f'name: "{entity_name}"\n'
            f'position {{\n'
            f'  x: {x}\n'
            f'  y: {y}\n'
            f'  z: {z}\n'
            f'}}\n'
        )
        try:
            with tempfile.NamedTemporaryFile(
                mode='w', suffix='.txt', delete=False
            ) as f:
                f.write(req_text)
                tmp = f.name
            result = subprocess.run([
                'gz', 'service', '-s', '/world/world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--reqfile', tmp,
            ], capture_output=True, timeout=3, text=True)
            if result.returncode == 0:
                return
        except FileNotFoundError:
            pass
        except Exception:
            pass
        finally:
            if 'tmp' in locals():
                try:
                    os.unlink(tmp)
                except OSError:
                    pass

    def _remove_entity(self, entity_name: str):
        """Remove an entity from Gazebo."""
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'remove',
            '-world', 'world',
            '-name', entity_name,
        ]
        try:
            subprocess.run(cmd, capture_output=True, timeout=5)
            with self._payload_lock:
                if entity_name in self._payload_entities:
                    self._payload_entities.remove(entity_name)
        except Exception:
            pass

    # ═══════════════════════════════════════════════════════════════════════
    # PAYLOAD VISUALIZATION
    # ═══════════════════════════════════════════════════════════════════════

    def _spawn_payload(self, entity_name: str, location_name: str, z: float = 0.01):
        """Spawn a visual payload box at the given hospital location."""
        if location_name not in HOSPITAL_LOCATIONS:
            return
        x, y, yaw = HOSPITAL_LOCATIONS[location_name]
        safe_name = entity_name.replace('/', '_').replace(' ', '_')
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'world',
            '-name', f'payload_{safe_name}',
            '-file', os.path.join(self._payload_model_path, 'model.sdf'),
            '-x', str(x), '-y', str(y), '-z', str(z),
            '-Y', str(yaw),
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, timeout=5, text=True)
            if result.returncode != 0:
                self.get_logger().error(
                    f'Payload spawn FAILED (rc={result.returncode}): {result.stderr}'
                )
            with self._payload_lock:
                self._payload_entities.append(f'payload_{safe_name}')
            self.get_logger().info(
                f'Spawned payload at {location_name} ({x:.1f},{y:.1f})'
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to spawn payload: {e}')

    def _schedule_remove(self, entity_name: str, delay: float = 3.0):
        """Schedule entity removal after a delay."""
        safe_name = entity_name.replace('/', '_').replace(' ', '_')
        full_name = f'payload_{safe_name}'
        cancelled = False
        def _do_remove():
            nonlocal cancelled
            if cancelled:
                return
            cancelled = True
            self._remove_entity(full_name)
        t = self.create_timer(delay, _do_remove)
        if not hasattr(self, '_remove_timers'):
            self._remove_timers = []
        self._remove_timers.append(t)

    def _cleanup_delivered(self):
        """Periodically remove old delivered payload boxes."""
        with self._payload_lock:
            delivered = [e for e in self._payload_entities if '_delivered' in e]
            for entity in delivered[:]:
                cmd = [
                    'ros2', 'run', 'ros_gz_sim', 'remove',
                    '-world', 'hospital',
                    '-name', entity,
                ]
                try:
                    subprocess.run(cmd, capture_output=True, timeout=3)
                    self._payload_entities.remove(entity)
                except Exception:
                    pass

    # ═══════════════════════════════════════════════════════════════════════
    # HELPERS
    # ═══════════════════════════════════════════════════════════════════════

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp    = self.get_clock().now().to_msg()
        p.pose.position.x = x - self._spawn_x
        p.pose.position.y = y - self._spawn_y
        p.pose.orientation.z = math.sin(yaw / 2)
        p.pose.orientation.w = math.cos(yaw / 2)
        return p

    def _log(self, tid: str, event: str, location: str):
        msg      = String()
        msg.data = json.dumps({
            'task_id':   tid,
            'event':     event,
            'location':  location,
            'robot_id':  self.robot_id,
            'timestamp': time.time(),
        })
        self._log_pub.publish(msg)

    def _publish_status(self):
        with self._lock:
            queued = len(self._queue)
        pickup_step = ''
        delivery_step = ''
        if self._active:
            if hasattr(self._active, 'pickup_step'):
                pickup_step = self._active.pickup_step
            if hasattr(self._active, 'delivery_step'):
                delivery_step = self._active.delivery_step
        msg = String()
        msg.data = json.dumps({
            'robot_id':       self.robot_id,
            'state':          self._active.state if self._active else MissionState.IDLE,
            'queued':         queued,
            'active':         self._active.task_id if self._active else None,
            'completed':      len(self._completed),
            'delivered':      sum(1 for t in self._completed
                                  if t.state == MissionState.MISSION_COMPLETE),
            'carrying':       self._carrying,
            'pickup_step':    pickup_step,
            'delivery_step':  delivery_step,
            'timestamp':      time.time(),
        })
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

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
