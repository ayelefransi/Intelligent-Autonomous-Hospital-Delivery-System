import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

class CostmapDumper(Node):
    def __init__(self):
        super().__init__('costmap_dumper')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.listener_callback,
            10)
        self.get_logger().info('Waiting for costmap...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received costmap: {msg.info.width}x{msg.info.height}')
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        # Convert costmap values (0-100, -1) to an image
        # -1 (unknown) -> 128 (gray)
        # 0 (free) -> 255 (white)
        # 100 (lethal) -> 0 (black)
        img = np.zeros_like(data, dtype=np.uint8)
        img[data == -1] = 128
        img[(data >= 0) & (data <= 100)] = 255 - (data[(data >= 0) & (data <= 100)] * 255 // 100)
        
        # Flip image to match standard map orientation (origin at bottom left in map, top left in image)
        img = np.flipud(img)
        
        # Save image
        cv2.imwrite('/tmp/costmap.png', img)
        self.get_logger().info('Saved /tmp/costmap.png')
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    costmap_dumper = CostmapDumper()
    try:
        rclpy.spin(costmap_dumper)
    except SystemExit:
        pass
    costmap_dumper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
