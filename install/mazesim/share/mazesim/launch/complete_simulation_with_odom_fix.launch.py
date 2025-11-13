import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    mazesim_dir = get_package_share_directory('mazesim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    world_file = os.path.join(mazesim_dir, 'worlds', 'maze_world.world')
    model_file = os.path.join(mazesim_dir, 'models', 'turtlebot3_burger', 'model.sdf')
    map_yaml_file = os.path.join(mazesim_dir, 'maps', 'my_map_1.yaml')

    os.environ['GAZEBO_MODEL_PATH'] = f"{os.path.join(mazesim_dir, 'models')}:{os.path.join(turtlebot3_gazebo_dir, 'models')}"
    
    # Robot configurations
    robots = [
        {'name': 'robot1', 'x': 0.0, 'y': 0.0, 'z': 0.01, 'yaw': 1.57, 'room': 'room_1'},
        {'name': 'robot3', 'x': -1.2, 'y': 1.0, 'z': 0.01, 'yaw': 0.0, 'room': 'room_3'},
    ]
    
    # Gazebo launch
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py'))
    )
    
    launch_list = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gzserver,
        gzclient
    ]
    
    for i, robot in enumerate(robots):
        robot_name = robot['name']
        nav2_params_file = os.path.join(mazesim_dir, 'config', f'nav2_params_{robot_name}.yaml')
        
        # Spawn robot in Gazebo
        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{robot_name}',
            arguments=[
                '-entity', robot_name,
                '-file', model_file,
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', str(robot['z']),
                '-Y', str(robot['yaw']),
                '-robot_namespace', robot_name,
            ],
            output='screen'
        )

        # Robot state publisher
        robot_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': True,
                'robot_description': open(model_file, 'r').read(),
                'frame_prefix': f'{robot_name}/',
            }],
            output='screen'
        )
        
        # CRITICAL: Laser frame remapper
        # This subscribes to /robotX/scan (from Gazebo with wrong frame_id)
        # and republishes to /robotX/scan_corrected with correct frame_id
        laser_remapper = Node(
            package='mazesim',
            executable='laser_frame_remapper.py',
            name=f'{robot_name}_laser_remapper',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot_name
            }],
            remappings=[
                ('scan_raw', 'scan'),        # Subscribe to Gazebo's scan
                ('scan', 'scan_corrected')   # Publish corrected scan
            ],
            output='screen'
        )
        
        # Odometry frame fixer
        odom_fixer = Node(
            package='mazesim',
            executable='odom_frame_fixer.py',
            name=f'{robot_name}_odom_frame_fixer',
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot_name
            }],
            remappings=[
                ('odom_raw', f'/{robot_name}/odom'),
                ('odom', f'/{robot_name}/odom_fixed')
            ],
            output='screen'
        )
        
        # Odometry TF broadcaster - MUST BE GLOBAL
        odom_tf_broadcaster = Node(
            package='mazesim',
            executable='odom_tf_broadcaster.py',
            name=f'{robot_name}_odom_tf_broadcaster',
            # NO NAMESPACE HERE - node handles it internally
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot_name
            }],
            # NO REMAPPINGS - node subscribes directly
            output='screen'
        )
        
        # Nav2 for this robot
        nav2_bringup = GroupAction([
            PushRosNamespace(robot_name),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'autostart': 'true',
                    'use_composition': 'True',
                    'namespace': robot_name,
                }.items()
            )
        ])
        
        # Command translator
        translator = Node(
            package='mazesim',
            executable='nav2_command_translator',
            name='nav2_command_translator',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot_name,
                'initial_room': robot['room']
            }]
        )

        # Map loader - triggers map loading after map_server starts
        map_loader = Node(
            package='mazesim',
            executable='map_loader.py',
            name=f'{robot_name}_map_loader',
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot_name,
                'map_yaml_path': map_yaml_file
            }],
            output='screen'
        )
        
        base_delay = 3.0 + (i * 3.0)
        launch_list.extend([
            TimerAction(period=base_delay, actions=[spawn]),
            TimerAction(period=base_delay + 1.0, actions=[robot_state_pub]),
            TimerAction(period=base_delay + 1.2, actions=[laser_remapper]),
            TimerAction(period=base_delay + 1.4, actions=[odom_fixer]),
            TimerAction(period=base_delay + 1.7, actions=[odom_tf_broadcaster]),
            TimerAction(period=base_delay + 2.0, actions=[nav2_bringup]),
            TimerAction(period=base_delay + 3.5, actions=[map_loader]),  # NEW: Load map after Nav2 starts
            TimerAction(period=base_delay + 4.0, actions=[translator])
        ])
            
    return LaunchDescription(launch_list)