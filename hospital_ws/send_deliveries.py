#!/usr/bin/env python3
"""Send delivery tasks matching the 7-task spec to the mission manager"""
import rclpy
from rclpy.node import Node  # type: ignore[import-unresolved]
from std_msgs.msg import String
import time
import json

rclpy.init()
node = Node("batch_delivery")

pub = node.create_publisher(String, "/delivery_request", 10)
print(f"Waiting for subscriber...")
for i in range(20):
    if pub.get_subscription_count() > 0:
        break
    time.sleep(0.3)

# Single task: pharmacy -> patient_room1 (morphine, STAT)
# Test one complete end-to-end delivery cycle
tasks = [
    {"origin": "pharmacy",      "destination": "patient_room1", "payload": "morphine",     "priority": "STAT"},
]

for task in tasks:
    msg = String()
    msg.data = json.dumps(task)
    pub.publish(msg)
    print(f"  -> {task['origin']} -> {task['destination']} [{task['priority']}] {task['payload']}")
    time.sleep(0.4)

print(f"\nSent {len(tasks)} delivery tasks!")
node.destroy_node()
rclpy.shutdown()
