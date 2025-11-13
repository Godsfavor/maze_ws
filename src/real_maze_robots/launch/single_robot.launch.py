from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot1',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            'initial_room',
            default_value='room_1',
            description='Starting room (room_1, room_2, room_3, or room_4)'
        ),
        
        Node(
            package='real_maze_robots',
            executable='real_robot_controller',
            name='real_robot_controller',
            output='screen',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'initial_room': LaunchConfiguration('initial_room'),
                'safe_distance': 0.30,
                'slow_distance': 0.50,
            }]
        )
    ])
