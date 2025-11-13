#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomFrameFixer(Node):
    def __init__(self):
        super().__init__('odom_frame_fixer')
        
        # Get the robot name (robot1, robot3, etc.)
        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').value
        self.robot_name = robot_name
        
        # Subscribe to raw odometry from Gazebo (has empty frame_id)
        self.subscription = self.create_subscription(
            Odometry,
            f'/{robot_name}/odom',  # Direct subscription to gazebo's odom
            self.odom_callback,
            10
        )
        
        # Publish fixed odometry
        self.publisher = self.create_publisher(
            Odometry,
            f'/{robot_name}/odom_fixed',  # Publish to a new topic
            10
        )
        
        self.get_logger().info(f'Odom Frame Fixer started for {robot_name}')
        self.get_logger().info(f'  Subscribing to: /{robot_name}/odom')
        self.get_logger().info(f'  Publishing to: /{robot_name}/odom_fixed')
    
    def odom_callback(self, msg):
        # Create fixed message
        fixed_msg = Odometry()
        fixed_msg.header.stamp = msg.header.stamp
        fixed_msg.header.frame_id = f'{self.robot_name}/odom'
        fixed_msg.child_frame_id = f'{self.robot_name}/base_footprint'
        
        # Copy all data
        fixed_msg.pose = msg.pose
        fixed_msg.twist = msg.twist
        
        self.publisher.publish(fixed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()