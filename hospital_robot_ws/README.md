# Intelligent Autonomous Hospital Delivery System
## TurtleBot4 + ROS 2 Jazzy + Gazebo Harmonic + Nav2 + SLAM Toolbox

Production-level autonomous hospital delivery robot.
Runs on **WSL2 Ubuntu 24.04 with WSLg** (built-in GUI).

---

## System Requirements

| Component      | Version              |
|----------------|----------------------|
| OS             | Ubuntu 24.04 on WSL2 |
| ROS 2          | Jazzy Jalisco        |
| Gazebo         | Harmonic             |
| Python         | 3.12+                |
| RAM            | 16 GB recommended    |
| WSLg           | Required for GUI     |

---

## Step 1: WSL2 and WSLg Setup

In PowerShell (Windows):
```powershell
wsl --install -d Ubuntu-24.04
wsl --update
```

Verify WSLg is working (inside WSL2):
```bash
echo $DISPLAY        # Should print :0
xclock               # Should open a window
```

---

## Step 2: Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 3: Install Gazebo Harmonic

```bash
sudo apt-get update
sudo apt-get install -y curl
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable \
  $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic
```

---

## Step 4: Install ROS-Gazebo Bridge and Nav2

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-msgs \
  ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-mppi-controller \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip

pip install scipy numpy --break-system-packages
```

---

## Step 5: Build the Workspace

```bash
cd ~/hospital_robot_ws
rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
echo "source ~/hospital_robot_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 6: Launch the System

### Single Robot (SLAM mode - builds map while navigating)
```bash
ros2 launch hospital_bringup hospital_full.launch.py mode:=slam
```

### Single Robot (Navigation mode - uses existing map)
```bash
ros2 launch hospital_bringup hospital_full.launch.py \
  mode:=nav map:=/path/to/your/map.yaml
```

### 3-Robot Fleet (Multi-robot with coordinator)
```bash
ros2 launch hospital_bringup hospital_multi_robot.launch.py
```

### Exploration only (autonomous SLAM mapping)
```bash
ros2 launch hospital_bringup hospital_full.launch.py \
  mode:=slam use_exploration:=true
```

---

## Step 7: Sending Delivery Tasks

### Via command line:
```bash
# Single STAT delivery (pharmacy to ICU)
ros2 topic pub --once /delivery_request std_msgs/msg/String \
  '{"data": "{\"origin\": \"pharmacy\", \"destination\": \"icu\", \"payload\": \"blood_products\", \"priority\": \"STAT\"}"}'

# Multi-robot global task
ros2 topic pub --once /hospital/delivery_request std_msgs/msg/String \
  '{"data": "{\"origin\": \"pharmacy\", \"destination\": \"patient_room_2\", \"payload\": \"medication\", \"priority\": \"URGENT\"}"}'
```

### Supported destinations:
```
pharmacy, icu, patient_room_1, patient_room_2, patient_room_3, patient_room_4,
nurse_station_1, nurse_station_2, nurse_station_3, lab, or_1, emergency, home
```

---

## Step 8: Save the SLAM Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/hospital_map
# Saves hospital_map.yaml and hospital_map.pgm
```

---

## Teleoperation

In a new terminal:
```bash
source ~/hospital_robot_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Algorithm Selection at Runtime

Switch global planner (A* vs Hybrid-A*):
```bash
ros2 service call /planner_server/change_state lifecycle_msgs/srv/ChangeState \
  '{transition: {id: 3}}'
```

Or use the Nav2 goal sender with planner override:
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 15.0, y: 0.0}, orientation: {w: 1.0}}}'
```

---

## Monitoring

```bash
# Robot status
ros2 topic echo /delivery_status

# Multi-robot fleet status
ros2 topic echo /hospital/system_status

# Tracked obstacles
ros2 topic echo /tracked_obstacles

# Exploration status
ros2 topic echo /exploration/status

# Task log
ros2 topic echo /task_log

# TF tree
ros2 run tf2_tools view_frames

# Nav2 lifecycle status
ros2 lifecycle list
```

---

## Troubleshooting

### Gazebo won't open (WSL2 GUI issue)
```bash
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1   # Use if GPU not available
gz sim --headless-rendering      # Last resort
```

### Nav2 not starting
```bash
# Check all Nav2 nodes are active
ros2 lifecycle list
# Look for 'inactive' nodes and activate them
ros2 lifecycle set /controller_server activate
```

### SLAM Toolbox not publishing map
```bash
# Verify scan topic is flowing
ros2 topic hz /scan
# Should show ~10 Hz
ros2 topic echo /scan --once
```

### Robot not moving
```bash
# Check cmd_vel is publishing
ros2 topic echo /cmd_vel
# Check diff drive bridge
ros2 topic list | grep cmd_vel
# Verify gz bridge is running
ros2 node list | grep bridge
```

### Obstacle tracker empty
```bash
# Verify LiDAR data
ros2 topic hz /scan
# Reduce detection range if needed
ros2 param set /obstacle_tracker max_range 3.0
```

### Build fails: missing package
```bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-jazzy-<missing-package>
```

---

## Package Structure

```
hospital_robot_ws/
  src/
    hospital_robot_description/    # TurtleBot4 hospital URDF (xacro)
      urdf/
        hospital_turtlebot4.urdf.xacro    # Top-level assembler
        components/
          base.xacro                      # Differential drive base
          sensors.xacro                   # LiDAR, IMU, RGB-D, ultrasonic
          payload.xacro                   # Delivery compartment
      launch/display.launch.py

    hospital_gazebo/               # Gazebo world and models
      worlds/hospital.world               # Full hospital SDF

    hospital_navigation/           # Nav2 + SLAM configuration
      config/
        nav2_params.yaml                  # Full Nav2 config (MPPI + Hybrid-A*)
        slam_toolbox_params.yaml          # SLAM Toolbox config
      behavior_trees/
        hospital_nav_bt.xml               # Hospital behavior tree

    hospital_mission/              # Python mission nodes
      hospital_mission/
        task_manager.py                   # Priority delivery queue + Nav2 client
        multi_robot_coordinator.py        # Hungarian algorithm fleet coordination
        obstacle_tracker.py               # DBSCAN + Kalman filter tracker
        frontier_explorer.py              # Frontier-based SLAM exploration

    hospital_bringup/              # Top-level launch files
      launch/
        hospital_full.launch.py           # Single robot full system
        hospital_multi_robot.launch.py    # 3-robot fleet

  report/
    hospital_robot_technical_report.pdf
```

---

## Performance Targets

| Metric             | Target     |
|--------------------|------------|
| Delivery success   | > 90%      |
| Avg delivery time  | < 60 sec   |
| Map accuracy       | < 0.15m drift |
| Obstacle avoidance | > 95%      |
| Fleet utilization  | > 70%      |

---

Built with ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04 WSL2 + WSLg.
