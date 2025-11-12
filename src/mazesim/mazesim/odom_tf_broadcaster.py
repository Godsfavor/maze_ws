#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster', namespace='')  # ← EMPTY NAMESPACE
        
        # Get the robot name
        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').value
        self.robot_name = robot_name
        
        # Create a TF broadcaster - publishes to GLOBAL /tf
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to the robot's odometry (this CAN be namespaced)
        self.subscription = self.create_subscription(
            Odometry,
            f'/{robot_name}/odom_fixed',
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f'Broadcasting {robot_name}/odom -> {robot_name}/base_footprint to GLOBAL /tf')
    
    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = f'{self.robot_name}/odom'
        t.child_frame_id = f'{self.robot_name}/base_footprint'
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()