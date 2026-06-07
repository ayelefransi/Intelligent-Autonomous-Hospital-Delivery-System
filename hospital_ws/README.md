# Autonomous Hospital Delivery Robot
### TurtleBot3 Waffle · ROS 2 Jazzy · Gazebo Harmonic · Nav2 · SLAM Toolbox · AWS RoboMaker Hospital World

Fully autonomous hospital delivery robot. Delivers medicines and supplies between
pharmacy, supply room, lab, reception, nurse station, and 5 patient rooms inside the
AWS RoboMaker hospital world.

**Three-phase delivery with robotic gripper hand:**
payload box appears at pickup → gripper hand grabs it → rides on robot → delivered at destination → robot returns home.

**Navigation:** Smac Hybrid-A* global planner + MPPI local controller.

**Speed & Design:** 15 m/s max linear, 40 rad/s angular. The robot has been completely customized: painted white, scaled 5x in height and 2x in width, and the gripper is mounted 1.0m high! Large bright-red payload boxes with medical crosses are visible during pickup and delivery. Includes automatic navigation retry logic.

**12-state mission machine:** IDLE → WAITING_FOR_TASK → TASK_ASSIGNED → NAVIGATING_TO_PICKUP → ARRIVED_AT_PICKUP → PICKING_UP_PAYLOAD → PAYLOAD_SECURED → NAVIGATING_TO_DESTINATION → ARRIVED_AT_DESTINATION → DELIVERING_PAYLOAD → RETURNING_HOME → MISSION_COMPLETE.

**Platform:** Docker with WSLg GPU passthrough (stable simulation clock).

---

## Quick Start

```bash
cd /mnt/c/Users/Hello/Music/hospital_robot_ws && docker build --no-cache -f /mnt/c/Users/Hello/Music/hospital_robot_ws/Dockerfile -t hospital-robot /mnt/c/Users/Hello/Music/hospital_robot_ws && ./run.sh
```

Gazebo opens. The robot spawns at `(-3.5, 1.0)` after 12 seconds. Move the Gazebo
camera to see it — right-click drag to orbit, scroll to zoom, or jump directly:

```bash
docker exec hospital-robot bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
   --reptype gz.msgs.Boolean --timeout 3000 \
   --req 'pose: {position: {x: -3.5, y: 1.0, z: 3.0}, look_at: {x: -3.5, y: 1.0, z: 0.2}}'"
```

The mission manager auto-submits 7 demo delivery tasks at t=23s. The robot navigates
to pickup locations, collects items (visual payload box appears on top), delivers
to destinations, then returns home after each mission.

---

## What is inside

```
hospital_robot_ws/
  Dockerfile                         ← Docker build (GPU passthrough + AWS world)
  entrypoint.sh                      ← Container entrypoint (sources ROS + sets paths)
  run.sh                             ← Build + launch script
  send_deliveries.py                 ← Batch delivery task sender (7 tasks)
  scripts/
    convert_aws_world.py             ← SDF 1.6→1.9 conversion
    fix_dae_materials.py             ← DAE → emissive_map PBR fix (OGRE2)
  hospital_ws/
    src/
      hospital_robot/                ← Main Python package
        hospital_robot/
          mission_manager.py         ← 12-state delivery machine + return-home + priority queue
          fleet_coordinator.py       ← Hungarian algorithm fleet dispatch
          obstacle_tracker.py        ← DBSCAN + Kalman tracker (Doctor/Nurse/Cart/Static)
          frontier_explorer.py       ← Frontier-based autonomous exploration
          dynamic_obstacles.py       ← Moving obstacles (cart, visitor, gurney)
          health_monitor.py          ← Sensor/localization/fleet health monitoring
        launch/
          hospital_slam.launch.py    ← Single robot, SLAM mode
          hospital_nav.launch.py     ← Single robot, pre-built map
          hospital_multi.launch.py   ← 3 robots + fleet coordinator
        config/
          nav2_params.yaml           ← Smac Hybrid-A* + MPPI controller + AMCL
          slam_toolbox_params.yaml   ← Online async SLAM
          hospital.rviz              ← RViz2 layout
        behavior_trees/
          hospital_bt.xml            ← Nav2 behavior tree with recovery actions

      hospital_world_bridge/         ← ament wrapper for hospital world assets
        worlds/                      ← hospital_clean.world + AWS hospital.world
        models/                      ← AWS hospital models (emissive_map PBR)
          payload_box/               ← Delivery payload box model
          gripper/                   ← Robotic gripper hand (two-finger parallel jaw)
```

---

## Launch sequence (timed)

| Time | Component |
|------|-----------|
| t=0  | Gazebo Harmonic + hospital world |
| t=3  | gz→ROS bridge (clock, scan, odom, cmd_vel, tf, imu) |
| t=5  | robot_state_publisher (TurtleBot3 Waffle URDF) |
| t=12 | Spawn robot via ros2 run ros_gz_sim create |
| t=14 | RViz2 (if use_rviz:=true) |
| t=15 | SLAM Toolbox (online async) |
| t=18 | Nav2 navigation stack (MPPI controller) |
| t=20 | Dynamic obstacles (hospital cart, visitor, gurney) |
| t=21 | Obstacle tracker (Doctor/Nurse/Cart/Static classification) |
| t=22 | Health monitor (sensor/localization/fleet monitoring) |
| t=23 | Mission manager (12-state machine, auto-submits 7 demo tasks) |
| t=26 | Frontier explorer (if use_explore:=true) |

---

## Launch arguments

```bash
ros2 launch hospital_robot hospital_slam.launch.py \
  use_rviz:=true            # Open RViz2
  use_explore:=false        # Frontier exploration (disable for deliveries)
  x_pose:=-3.5              # Robot spawn X
  y_pose:=1.0               # Robot spawn Y
  yaw:=0.0                  # Robot spawn yaw
```

---

## Delivery system

### Three-phase workflow

```
Task received → Navigate to PICKUP (origin) → Pick up item
                                             → Gripper grabs payload
              → Navigate to DELIVER (destination) → Drop item
                                                  → Box spawns at destination
              → Navigate to HOME (-3.5, 1.0)      → MISSION_COMPLETE
```

Each task goes through the priority queue three times — once per phase (pickup,
delivery, return home). Priority ordering is preserved: a STAT task's delivery
phase beats a ROUTINE task's pickup phase.

### 12-state mission machine

```
IDLE → WAITING_FOR_TASK → TASK_ASSIGNED → NAVIGATING_TO_PICKUP → ARRIVED_AT_PICKUP
→ PICKING_UP_PAYLOAD → PAYLOAD_SECURED → NAVIGATING_TO_DESTINATION
→ ARRIVED_AT_DESTINATION → DELIVERING_PAYLOAD → RETURNING_HOME → MISSION_COMPLETE
```

Also: `MISSION_FAILED`, `CANCELLED`

### Visual payload and gripper

- **Pickup location**: red payload box spawns when robot arrives (confirms item was there)
- **Gripper**: robotic two-finger hand spawns on robot (z=0.35m), gripping the payload during transit — follows odometry at 2Hz
- **Delivery**: gripper released, payload box spawns at destination, auto-cleaned after 15s
- **Return home**: robot navigates back to home (-3.5, 1.0) after delivery

### Priority levels

- **STAT (0)**: Emergency — morphine, blood products
- **URGENT (1)**: Time-sensitive — antibiotics, IV drips, linens
- **ROUTINE (2)**: Standard — bandages, blood samples, visitor passes

### Demo tasks (auto-submitted)

| # | Origin | Destination | Payload | Priority |
|---|--------|-------------|---------|----------|
| 1 | pharmacy | patient_room1 | morphine | STAT |
| 2 | pharmacy | patient_room2 | antibiotics | URGENT |
| 3 | supply_room | nurse_station | bandages | ROUTINE |
| 4 | pharmacy | patient_room3 | IV_drip | URGENT |
| 5 | lab | nurse_station | blood_sample | ROUTINE |
| 6 | reception | patient_room4 | visitor_pass | ROUTINE |
| 7 | supply_room | patient_room5 | linens | URGENT |

### Send tasks manually

```bash
# Batch (all 7)
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/hospital_ws/install/setup.bash && \
   python3 ~/hospital_ws/send_deliveries.py'
```

```bash
# Single via ROS topic
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && \
   ros2 topic pub --once /delivery_request std_msgs/msg/String \
   "{data: \"{\\\"origin\\\": \\\"pharmacy\\\", \\\"destination\\\": \\\"patient_room1\\\", \\\"payload\\\": \\\"morphine\\\", \\\"priority\\\": \\\"STAT\\\"}\"}"'
```

### Available locations

| Location | Coordinates (x, y, yaw) | Description |
|----------|------------------------|-------------|
| reception | (0.0, -5.5, 0.0) | Waiting area |
| pharmacy | (9.0, 10.0, 0.0) | Right north wing |
| supply_room | (-10.0, 10.0, 3.14) | Left north wing |
| patient_room1 | (11.0, -2.0, 1.57) | Right wing top |
| patient_room2 | (11.0, -7.0, 1.57) | Right wing mid |
| patient_room3 | (11.0, -18.0, 1.57) | Right wing bottom |
| patient_room4 | (-11.0, 0.0, -1.57) | Left wing top |
| patient_room5 | (-11.0, -12.0, -1.57) | Left wing bottom |
| nurse_station | (0.0, 1.5, 0.0) | Center |
| lab | (-1.0, -21.0, 0.0) | South |
| home | (-3.5, 1.0, 0.0) | Robot spawn/dock |

---

## Obstacle classification

The obstacle tracker classifies dynamic obstacles using DBSCAN clustering + Kalman
filtering, with distinct types and robot responses per the hospital safety protocol:

| Type | Detection | Robot Response |
|------|-----------|----------------|
| **Doctor** | Small radius, speed > 0.6 m/s (fast, unpredictable) | `YIELD` — yield immediately, slow down, increase safety margin |
| **Nurse** | Small radius, speed 0.05–0.6 m/s (semi-predictable) | `MAINTAIN_CLEARANCE` — maintain clearance, continue if safe |
| **Cart** | Larger radius (0.5–0.9m), moving | `REPLAN` — replan route if required |
| **Static** | Not moving (beds, equipment, furniture) | `NAVIGATE_AROUND` — navigate around obstacle |

Published to `/tracked_obstacles` (JSON) and visualized via `/obstacle_markers` (RViz).

---

## Health monitoring

The health monitor node continuously checks robot subsystem health:

| Component | Source Topic | Timeout → Severity |
|-----------|-------------|--------------------|
| LiDAR | `/scan` | 5s → CRITICAL |
| Odometry | `/odom` | 5s → CRITICAL |
| Mission Manager | `/mission_status` | 10s → WARNING |
| Fleet | `/hospital/fleet_status` | 20s → WARNING |

**Critical failure response:** Stop robot (zero cmd_vel) → Report error → Await recovery.

Health status published to `/robot_health` at 1 Hz.

---

## Monitoring

```bash
# Delivery status (live) — shows 12-state transitions
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /mission_status'

# Task log
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /task_log'

# Robot health (sensor/localization monitoring)
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /robot_health'

# Obstacle tracking (Doctor/Nurse/Cart/Static classification)
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /tracked_obstacles'

# Gazebo model list (confirm robot exists)
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && gz model --list'

# Robot pose
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && gz model -m turtlebot3_waffle -p'

# Container logs
docker logs -f hospital-robot
```

---

## Troubleshooting

### Robot not visible in Gazebo

The robot spawns at `(-3.5, 1.0)` — far from the default camera. Move the camera:
```bash
docker exec hospital-robot bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
   --reptype gz.msgs.Boolean --timeout 3000 \
   --req 'pose: {position: {x: -3.5, y: 1.0, z: 3.0}, look_at: {x: -3.5, y: 1.0, z: 0.2}}'"
```

Or right-click-drag in Gazebo to orbit, scroll to zoom, shift-drag to pan.

### Robot not moving / Nav2 stuck

```bash
# Check bt_navigator state
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /bt_navigator'

# If inactive [2], activate:
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && \
   ros2 service call /bt_navigator/change_state lifecycle_msgs/srv/ChangeState \
   "{transition: {id: 3, label: \"activate\"}}"'
```

### SLAM not publishing /map

SLAM auto-activates at t=25s via launch file TimerAction. If it didn't:
```bash
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/hospital_ws/install/setup.bash && \
   ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState \
   "{transition: {id: 1, label: \"configure\"}}" && sleep 3 && \
   ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState \
   "{transition: {id: 3, label: \"activate\"}}"'
```

### Hospital world is dark

All models use `emissive_map` (self-illuminating PBR) — no external lights needed.
If still dark, check renderer:
```bash
grep GL_RENDERER ~/.gz/rendering/ogre2.log
# llvmpipe = software rendering (emissive_map handles this)
```

### Cleanup before relaunch

```bash
docker stop hospital-robot
# Or if hung:
docker kill hospital-robot
```

---

## Testing

### Verify speed (2x)

After launch, check the robot is moving at the new 1.0 m/s max:

```bash
# Watch cmd_vel output — should peak at ~1.0 m/s during navigation
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /cmd_vel'
```

### Verify gripper hand

During a delivery task, check Gazebo for the gripper entity:

```bash
# List all models in Gazebo — look for gripper_{task_id}_carried during transit
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && gz model --list'
```

Expected output during transit phase:
```
turtlebot3_waffle
payload_robot_1_T0001   ← payload box at pickup location (disappears after 3s)
gripper_robot_1_T0001_carried   ← gripper hand on robot (during transit)
```

### Full end-to-end test

```bash
# 1. Rebuild and launch
cd /mnt/c/Users/Hello/Music/hospital_robot_ws
docker build --no-cache \
  -f /mnt/c/Users/Hello/Music/hospital_robot_ws/Dockerfile \
  -t hospital-robot \
  /mnt/c/Users/Hello/Music/hospital_robot_ws
./run.sh

# 2. Wait for robot spawn (12s) + SLAM activation (15s+)
# 3. Jump camera to robot:
docker exec hospital-robot bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
   --reptype gz.msgs.Boolean --timeout 3000 \
   --req 'pose: {position: {x: -3.5, y: 1.0, z: 3.0}, look_at: {x: -3.5, y: 1.0, z: 0.2}}'"

# 4. Watch 12-state mission transitions
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /mission_status'

# 5. Track speed during navigation (MPPI controller, max 1.0 m/s)
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /cmd_vel --field linear.x'

# 6. Verify gripper appears during transit phase
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && gz model --list | grep gripper'

# 7. Verify robot health monitoring
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /robot_health'

# 8. Verify obstacle classification (Doctor/Nurse/Cart/Static)
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /tracked_obstacles'

# 9. Manual delivery test (optional)
docker exec hospital-robot bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/hospital_ws/install/setup.bash && \
   ros2 topic pub --once /delivery_request std_msgs/msg/String \
   "data: \"{\\\"origin\\\": \\\"pharmacy\\\", \\\"destination\\\": \\\"nurse_station\\\", \\\"payload\\\": \\\"morphine\\\", \\\"priority\\\": \\\"STAT\\\"}\""'
```

## Docker build

```bash
docker build --no-cache \
  -f /mnt/c/Users/Hello/Music/hospital_robot_ws/Dockerfile \
  -t hospital-robot \
  /mnt/c/Users/Hello/Music/hospital_robot_ws
```

Docker run flags: `--privileged` (GPU), `--network host` (ROS 2 DDS), `--device=/dev/dri` (WSLg GPU), `-v /tmp/.X11-unix` (X11 forwarding).

---

## File locations

| File | Path |
|------|------|
| Dockerfile | `/mnt/c/Users/Hello/Music/hospital_robot_ws/Dockerfile` |
| Run script | `/mnt/c/Users/Hello/Music/hospital_robot_ws/run.sh` |
| Entrypoint | `/mnt/c/Users/Hello/Music/hospital_robot_ws/entrypoint.sh` |
| Mission manager | `.../hospital_ws/src/hospital_robot/hospital_robot/mission_manager.py` |
| Fleet coordinator | `.../hospital_ws/src/hospital_robot/hospital_robot/fleet_coordinator.py` |
| Obstacle tracker | `.../hospital_ws/src/hospital_robot/hospital_robot/obstacle_tracker.py` |
| Health monitor | `.../hospital_ws/src/hospital_robot/hospital_robot/health_monitor.py` |
| Frontier explorer | `.../hospital_ws/src/hospital_robot/hospital_robot/frontier_explorer.py` |
| Dynamic obstacles | `.../hospital_ws/src/hospital_robot/hospital_robot/dynamic_obstacles.py` |
| Launch file (SLAM) | `.../hospital_ws/src/hospital_robot/launch/hospital_slam.launch.py` |
| Launch file (Nav) | `.../hospital_ws/src/hospital_robot/launch/hospital_nav.launch.py` |
| Launch file (Multi) | `.../hospital_ws/src/hospital_robot/launch/hospital_multi.launch.py` |
| Delivery sender | `.../hospital_ws/send_deliveries.py` |
| Nav2 params (MPPI) | `.../hospital_robot/config/nav2_params.yaml` |
| SLAM params | `.../hospital_robot/config/slam_toolbox_params.yaml` |
| Behavior tree | `.../hospital_robot/behavior_trees/hospital_bt.xml` |
| Payload model | `.../hospital_world_bridge/models/payload_box/` |
| Gripper model | `.../hospital_world_bridge/models/gripper/` |
| AWS world (converted) | `.../hospital_world_bridge/worlds/hospital.world` |
| DAE fix script | `.../scripts/fix_dae_materials.py` |
| WSL workspace | `/home/fransi/hospital_ws/` |
| Git repo | `/mnt/c/Users/Hello/Music/hospital_robot_ws/` |
