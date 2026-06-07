#!/bin/bash
# Delivery workflow — runs inside Docker container
# Phase 1: Wait for system startup
# Phase 2: Activate SLAM
# Phase 3: Wait for map
# Phase 4: Send deliveries
source /opt/ros/jazzy/setup.bash
source /home/robot/hospital_ws/install/setup.bash

echo "=== Phase 1: Waiting for system startup (100s) ==="
sleep 100

echo "=== Phase 2: Activating SLAM ==="
ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3, label: 'activate'}}" 2>&1
sleep 3

echo "=== Phase 3: Waiting for map (60s) ==="
for i in $(seq 1 12); do
    sleep 5
    echo "  tick $((i*5))s"
done

echo "=== Phase 4: Sending deliveries ==="
python3 /home/robot/hospital_ws/send_deliveries.py

echo "=== Deliveries sent! Monitoring... ==="
sleep 300
echo "=== Done ==="
