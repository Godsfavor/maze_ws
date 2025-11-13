from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='real_maze_robots',
            executable='command_publisher',
            name='command_publisher',
            output='screen'
        )
    ])
