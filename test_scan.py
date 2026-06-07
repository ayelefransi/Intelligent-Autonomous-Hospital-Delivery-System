import rclpy
from sensor_msgs.msg import LaserScan
def cb(msg):
    hits = [r for r in msg.ranges if r != float('inf') and r > 0.0]
    if hits:
        print(f'Min range: {min(hits):.3f}')
        print(f'Hits < 0.5m: {[r for r in hits if r < 0.5]}')
    else:
        print('No hits')
    rclpy.shutdown()
rclpy.init()
node = rclpy.create_node('test_scan')
node.create_subscription(LaserScan, '/scan', cb, 1)
rclpy.spin(node)
