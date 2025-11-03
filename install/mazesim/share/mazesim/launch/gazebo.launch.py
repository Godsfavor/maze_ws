import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    mazesim_dir = get_package_share_directory('mazesim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Paths
    world_file = os.path.join(mazesim_dir, 'worlds', 'maze_world.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Set Gazebo model path to include our models
    gazebo_model_path = os.path.join(mazesim_dir, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    
    # Gazebo server (physics simulation)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        
        gzserver,
        gzclient,
    ])