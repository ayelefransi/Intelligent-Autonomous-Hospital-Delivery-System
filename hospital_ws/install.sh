#!/usr/bin/env bash
# =============================================================================
# Hospital Delivery Robot - WSL2 Ubuntu 24.04 Installer
# =============================================================================
# Installs:
#   - ROS 2 Jazzy
#   - Gazebo Harmonic
#   - ros_gz bridge
#   - Nav2 full stack
#   - SLAM Toolbox
#   - TurtleBot3 packages
#   - AWS RoboMaker Hospital World (cloned & ported to Gazebo Harmonic)
#   - This workspace
# =============================================================================
set -e

WS_DIR="$HOME/hospital_ws"
HOSPITAL_WORLD_REPO="https://github.com/aws-robotics/aws-robomaker-hospital-world.git"
TURTLEBOT3_MODEL="waffle"

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GRN}[INFO]${NC} $1"; }
warn()  { echo -e "${YLW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERR ]${NC} $1"; exit 1; }

# ── 0. Verify WSL2 + WSLg ────────────────────────────────────────────────────
info "Checking WSL2 + WSLg..."
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
    echo "export DISPLAY=:0" >> ~/.bashrc
fi
if ! command -v xdpyinfo &>/dev/null; then
    sudo apt-get install -y x11-utils 2>/dev/null || true
fi
info "Display: $DISPLAY"

# ── 1. System packages ───────────────────────────────────────────────────────
info "Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y \
    software-properties-common \
    curl wget git python3-pip python3-dev \
    build-essential cmake \
    libgl1 libglx-mesa0 libgl1-mesa-dri \
    x11-apps mesa-utils

# ── 2. ROS 2 Jazzy ───────────────────────────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    info "Installing ROS 2 Jazzy..."
    sudo apt-get install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu \
        $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y ros-jazzy-desktop ros-dev-tools
else
    info "ROS 2 Jazzy already installed"
fi

source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc 2>/dev/null || true

# ── 3. Gazebo Harmonic ───────────────────────────────────────────────────────
if ! command -v gz &>/dev/null; then
    info "Installing Gazebo Harmonic..."
    sudo curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable \
        $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y gz-harmonic
else
    info "Gazebo Harmonic already installed"
fi

# ── 4. ROS-Gazebo bridge + Nav2 + SLAM + TurtleBot3 ─────────────────────────
info "Installing ROS packages..."
sudo apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-mppi-controller \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-collision-monitor \
    ros-jazzy-slam-toolbox \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-navigation2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-ros \
    python3-colcon-common-extensions \
    python3-rosdep

pip3 install scipy numpy --break-system-packages

# ── 5. Clone AWS Hospital World ──────────────────────────────────────────────
HOSPITAL_SRC="$HOME/aws-robomaker-hospital-world"
if [ ! -d "$HOSPITAL_SRC" ]; then
    info "Cloning AWS RoboMaker Hospital World..."
    git clone $HOSPITAL_WORLD_REPO "$HOSPITAL_SRC"
else
    info "Hospital world already cloned at $HOSPITAL_SRC"
    cd "$HOSPITAL_SRC" && git pull --quiet
fi

# ── 6. Copy hospital world into our workspace ────────────────────────────────
info "Setting up hospital world in workspace..."
BRIDGE_DIR="$WS_DIR/src/hospital_world_bridge"
cp -r "$HOSPITAL_SRC/worlds"      "$BRIDGE_DIR/"
cp -r "$HOSPITAL_SRC/models"      "$BRIDGE_DIR/"
cp -r "$HOSPITAL_SRC/fuel_models" "$BRIDGE_DIR/" 2>/dev/null || true

# ── 7. rosdep init ───────────────────────────────────────────────────────────
info "Running rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y || true

# ── 8. Build workspace ───────────────────────────────────────────────────────
info "Building workspace..."
cd "$WS_DIR"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_cohesion+

# ── 9. Environment setup ─────────────────────────────────────────────────────
SETUP_LINE="source $WS_DIR/install/setup.bash"
grep -qxF "$SETUP_LINE" ~/.bashrc || echo "$SETUP_LINE" >> ~/.bashrc

TB3_LINE="export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
grep -qxF "$TB3_LINE" ~/.bashrc || echo "$TB3_LINE" >> ~/.bashrc

GZ_MODEL_LINE="export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:$HOSPITAL_SRC/models:$HOSPITAL_SRC/fuel_models"
grep -qxF "$GZ_MODEL_LINE" ~/.bashrc || echo "$GZ_MODEL_LINE" >> ~/.bashrc

info ""
info "=========================================="
info " Installation complete!"
info "=========================================="
info ""
info " Run: source ~/.bashrc"
info " Then: ros2 launch hospital_robot hospital_slam.launch.py"
info " Multi-robot: ros2 launch hospital_robot hospital_multi.launch.py"
info ""
