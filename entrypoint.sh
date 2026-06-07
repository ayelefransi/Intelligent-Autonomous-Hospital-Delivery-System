#!/bin/bash
# Entrypoint — source ROS, workspace, and Gazebo model paths
source /opt/ros/jazzy/setup.bash
source /home/robot/hospital_ws/install/setup.bash

# Make Gazebo find the hospital models (installed share + source fallback)
export GZ_SIM_RESOURCE_PATH="/home/robot/hospital_ws/install/hospital_world_bridge/share/hospital_world_bridge/models:/home/robot/hospital_ws/install/hospital_world_bridge/share/hospital_world_bridge/fuel_models:/home/robot/hospital_ws/src/hospital_world_bridge/models:/home/robot/hospital_ws/src/hospital_world_bridge/fuel_models:${GZ_SIM_RESOURCE_PATH}"
export TURTLEBOT3_MODEL=waffle
export DISPLAY=${DISPLAY:-:0}

exec "$@"
