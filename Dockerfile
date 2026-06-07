# Hospital Delivery Robot — Docker with AWS RoboMaker Hospital World
# Build:  docker build -t hospital-robot /home/fransi/hospital_ws
# Run:    docker run -it --rm --privileged --network host \
#           -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
#           --device=/dev/dri hospital-robot \
#           ros2 launch hospital_robot hospital_slam.launch.py world:=aws

FROM osrf/ros:jazzy-desktop-full

# ── System packages ────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    -o Acquire::Retries=5 \
    -o Acquire::ForceIPv4=true \
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
    python3-rosdep \
    python3-pip \
    mesa-utils \
    git \
    && rm -rf /var/lib/apt/lists/*

# Python deps
RUN pip3 install --break-system-packages scipy numpy

# Fix: turtlebot3 model references model://turtlebot3_common/meshes/...
# but meshes are shipped by turtlebot3_description. Symlink so Gazebo finds them.
RUN ln -sf /opt/ros/jazzy/share/turtlebot3_description \
          /opt/ros/jazzy/share/turtlebot3_common

# ── User setup ────────────────────────────────────────────────────────────────
ARG USER=robot
RUN useradd -m -s /bin/bash $USER && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USER
WORKDIR /home/$USER

# ── Environment ───────────────────────────────────────────────────────────────
ENV TURTLEBOT3_MODEL=waffle
ENV DISPLAY=:0
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# ── Copy pre-cloned AWS RoboMaker Hospital World ────────────────────────────
COPY --chown=$USER aws-robomaker-hospital-world/ /home/$USER/aws-robomaker-hospital-world/

# ── Copy workspace source ─────────────────────────────────────────────────────
# Build context is the hospital_robot_ws directory
COPY --chown=$USER hospital_ws/src/     ./hospital_ws/src/
COPY --chown=$USER hospital_ws/install.sh ./hospital_ws/install.sh
COPY --chown=$USER hospital_ws/send_deliveries.py ./hospital_ws/send_deliveries.py

# ── Copy conversion scripts ──────────────────────────────────────────────────
COPY --chown=$USER scripts/ ./hospital_ws/scripts/

# ── Populate world bridge with AWS hospital assets ────────────────────────────
RUN mkdir -p /home/$USER/hospital_ws/src/hospital_world_bridge/worlds && \
    cp /home/$USER/aws-robomaker-hospital-world/worlds/hospital.world \
       /home/$USER/hospital_ws/src/hospital_world_bridge/worlds/ && \
    cp -r /home/$USER/aws-robomaker-hospital-world/models \
          /home/$USER/hospital_ws/src/hospital_world_bridge/ && \
    (cp -r /home/$USER/aws-robomaker-hospital-world/fuel_models \
           /home/$USER/hospital_ws/src/hospital_world_bridge/ 2>/dev/null || true)

# ── Convert AWS world to Gazebo Harmonic (SDF 1.6→1.9, strip fuel models) ────
RUN python3 /home/$USER/hospital_ws/scripts/convert_aws_world.py \
    /home/$USER/hospital_ws/src/hospital_world_bridge/worlds

# ── Fix DAE model materials for OGRE2 (prevent gray rendering) ────────────────
RUN python3 /home/$USER/hospital_ws/scripts/fix_dae_materials.py \
    /home/$USER/hospital_ws/src/hospital_world_bridge

# ── Set GZ resource path for Gazebo to find hospital models ──────────────────
ENV GZ_SIM_RESOURCE_PATH=/home/$USER/hospital_ws/src/hospital_world_bridge/models:/home/$USER/hospital_ws/src/hospital_world_bridge/fuel_models

# ── Customize robot: 4x scale + completely white ─────────────────────────────
RUN sudo python3 /home/$USER/hospital_ws/scripts/customize_robot.py \
    /opt/ros/jazzy/share/turtlebot3_gazebo

# ── Build workspace ───────────────────────────────────────────────────────────
RUN /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    cd ~/hospital_ws && \
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY --chown=$USER entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
