import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    # Get package directory
    mazesim_dir = get_package_share_directory('mazesim')
    
    # Load room positions
    config_file = os.path.join(mazesim_dir, 'config', 'room_positions.yaml')
    
    # Define robot starting positions (from room_positions.yaml)
    robots = [
        {'name': 'robot1', 'x': 3.81, 'y': 16.10, 'z': 0.01, 'yaw': 0.0},
        {'name': 'robot2', 'x': 16.31, 'y': 16.10, 'z': 0.01, 'yaw': 0.0},
        {'name': 'robot3', 'x': 3.81, 'y': 3.60, 'z': 0.01, 'yaw': 0.0},
        {'name': 'robot4', 'x': 16.31, 'y': 3.60, 'z': 0.01, 'yaw': 0.0},
    ]
    
    launch_description_content = []
    
    # Spawn each robot
    for robot in robots:
        # Spawn robot in Gazebo
        spawn_robot = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', robot['name'],
                '-topic', 'robot_description',
                '-robot_namespace', robot['name'],
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', str(robot['z']),
                '-Y', str(robot['yaw'])
            ],
            output='screen'
        )
        launch_description_content.append(spawn_robot)
    
    return LaunchDescription(launch_description_content)
