#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
        
        # Declare parameter
        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').value
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            LaserScan,
            'scan_raw',
            self.scan_callback,
            10)
        
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        
        # Set the corrected frame name
        self.corrected_frame = f'{robot_name}/base_scan'
        
        self.get_logger().info(f'Scan frame fixer started for {robot_name}')
        self.get_logger().info(f'Will remap scan frame to: {self.corrected_frame}')
    
    def scan_callback(self, msg):
        # Fix the frame_id
        msg.header.frame_id = self.corrected_frame
        # Republish
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()