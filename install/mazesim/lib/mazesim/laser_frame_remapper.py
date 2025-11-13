#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserFrameRemapper(Node):
    def __init__(self):
        super().__init__('laser_frame_remapper')
        
        # Get robot name parameter
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value
        
        # Create subscriber to raw scan (from Gazebo)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan_raw',
            self.scan_callback,
            10
        )
        
        # Create publisher for remapped scan
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Target frame name
        self.target_frame = f'{self.robot_name}/base_scan'
        
        self.get_logger().info(f'Laser frame remapper started for {self.robot_name}')
        self.get_logger().info(f'Remapping frame_id to: {self.target_frame}')
    
    def scan_callback(self, msg):
        # Create new message with corrected frame_id
        remapped_msg = LaserScan()
        remapped_msg.header = msg.header
        remapped_msg.header.frame_id = self.target_frame  # Fix the frame_id
        remapped_msg.angle_min = msg.angle_min
        remapped_msg.angle_max = msg.angle_max
        remapped_msg.angle_increment = msg.angle_increment
        remapped_msg.time_increment = msg.time_increment
        remapped_msg.scan_time = msg.scan_time
        remapped_msg.range_min = msg.range_min
        remapped_msg.range_max = msg.range_max
        remapped_msg.ranges = msg.ranges
        remapped_msg.intensities = msg.intensities
        
        # Publish remapped scan
        self.scan_pub.publish(remapped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFrameRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
