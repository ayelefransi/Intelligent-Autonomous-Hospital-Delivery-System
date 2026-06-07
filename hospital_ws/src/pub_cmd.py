import rclpy
from geometry_msgs.msg import Twist
import time

rclpy.init()
node = rclpy.create_node('move_bot')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
msg = Twist()
msg.linear.x = 0.5
pub.publish(msg)
time.sleep(2.0)
msg.linear.x = 0.0
pub.publish(msg)
rclpy.shutdown()
