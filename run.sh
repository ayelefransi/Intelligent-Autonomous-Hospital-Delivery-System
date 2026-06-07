#!/bin/bash
# Hospital Robot — Docker on WSL2 (uses WSLg GPU passthrough)
# World: AWS RoboMaker Hospital World (only)
set -e
cd "$(dirname "$0")"

echo "=== Using AWS RoboMaker Hospital World ==="

# Build context: the repo root
WSL_WS="$(dirname "$0")"

if [ ! -d "$WSL_WS" ]; then
    echo "ERROR: Workspace not found at $WSL_WS"
    exit 1
fi

if [ "$1" == "--build" ]; then
    echo "=== Building Docker image from $WSL_WS ==="
    docker build \
        -f "$(dirname "$0")/Dockerfile" \
        -t hospital-robot \
        "$WSL_WS"
else
    echo "=== Skipping Docker build (run with --build to rebuild) ==="
fi

echo "=== Launching with WSLg GPU ==="
docker rm -f hospital-robot 2>/dev/null || true

docker run --rm \
    --name hospital-robot \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /mnt/wslg:/mnt/wslg:ro \
    -v /usr/lib/wsl:/usr/lib/wsl:ro \
    -v "$(pwd)/hospital_ws/src/hospital_robot:/home/robot/hospital_ws/src/hospital_robot:rw" \
    --device=/dev/dri \
    hospital-robot \
    ros2 launch hospital_robot hospital_slam.launch.py use_explore:=false x_pose:=-3.0 y_pose:=2.0
