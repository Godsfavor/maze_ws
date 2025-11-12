import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mazesim_dir = get_package_share_directory('mazesim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths
    map_yaml_file = os.path.join(mazesim_dir, 'maps', 'my_map_1.yaml')
    nav2_params_file = os.path.join(mazesim_dir, 'config', 'nav2_params.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace')
    
    # Nav2 bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'namespace': namespace
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_namespace_cmd,
        nav2_bringup_cmd
    ])