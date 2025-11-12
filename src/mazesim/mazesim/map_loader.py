#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
import time

class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')
        
        # Get parameters
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('map_yaml_path', '')
        
        robot_name = self.get_parameter('robot_name').value
        map_path = self.get_parameter('map_yaml_path').value
        
        self.get_logger().info(f'Map loader for {robot_name}')
        self.get_logger().info(f'Map path: {map_path}')
        
        # Create service client
        service_name = f'/{robot_name}/map_server/load_map'
        self.client = self.create_client(LoadMap, service_name)
        
        # Wait for service
        self.get_logger().info(f'Waiting for {service_name}...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for map_server service...')
        
        # Load the map
        self.load_map(map_path)
    
    def load_map(self, map_path):
        request = LoadMap.Request()
        request.map_url = map_path
        
        self.get_logger().info(f'Loading map from: {map_path}')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Map loaded successfully!')
        else:
            self.get_logger().error('Failed to load map!')

def main(args=None):
    rclpy.init(args=args)
    node = MapLoader()
    # Keep node alive briefly then shut down
    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
