import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    mazesim_dir = get_package_share_directory('mazesim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    world_file = os.path.join(mazesim_dir, 'worlds', 'maze_world.world')
    model_file = os.path.join(mazesim_dir, 'models', 'turtlebot3_burger', 'model.sdf')
    map_yaml_file = os.path.join(mazesim_dir, 'maps', 'my_map_1.yaml')
    
    # Gazebo
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py'))
    )
    
    # Static map->robot1/odom transform (temporary solution)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot1/odom']
    )
    
    # Robot 1
    spawn1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot1', '-file', model_file, '-x', '0', '-y', '0', '-z', '0.01', '-Y', '1.57', '-robot_namespace', 'robot1'],
        output='screen'
    )
    
    robot_state_pub1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[{'use_sim_time': True, 'robot_description': open(model_file).read(), 'frame_prefix': 'robot1/'}],
    )
    
    # Simplified Nav2 params inline
    nav2_params = {
        'use_sim_time': True,
        'yaml_filename': map_yaml_file,
        'autostart': True,
    }
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'true',
            'params_file': os.path.join(mazesim_dir, 'config', 'nav2_params_robot1.yaml'),
            'namespace': 'robot1',
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gzserver,
        gzclient,
        TimerAction(period=3.0, actions=[spawn1]),
        TimerAction(period=4.0, actions=[robot_state_pub1]),
        TimerAction(period=5.0, actions=[static_tf]),
        TimerAction(period=6.0, actions=[nav2]),
    ])