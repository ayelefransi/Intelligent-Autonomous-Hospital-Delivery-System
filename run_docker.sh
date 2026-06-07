#!/bin/bash
# Hospital Robot Docker entrypoint
set -e

# Allow X11 access
xhost +local:docker 2>/dev/null || true

# Start container with GPU + GUI
docker run -it --rm \
    --privileged \
    --gpus all \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.gz:/home/robot/.gz \
    hospital-robot \
    bash -c "source /opt/ros/jazzy/setup.bash && source ~/hospital_ws/install/setup.bash && ros2 launch hospital_robot hospital_slam.launch.py"
