import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    mazesim_dir = get_package_share_directory('mazesim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    world_file = os.path.join(mazesim_dir, 'worlds', 'maze_world.world')
    model_file = os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_burger', 'model.sdf')
    
    # Set model paths (for Classic Gazebo)
    os.environ['GAZEBO_MODEL_PATH'] = f"{os.path.join(mazesim_dir, 'models')}:{os.path.join(turtlebot3_gazebo_dir, 'models')}"
    
    # Robot configurations - spawn DIRECTLY at final positions
    # robots = [
    #     {'name': 'robot1', 'x': 4.06, 'y': 14.85, 'z': 0.01, 'yaw': 1.57, 'room': 'room_1'},
    #     {'name': 'robot2', 'x': 15.56, 'y': 14.85, 'z': 0.01, 'yaw': 3.14, 'room': 'room_2'},
    #     {'name': 'robot3', 'x': 4.06, 'y': 3.85, 'z': 0.01, 'yaw': 0.0, 'room': 'room_3'},
    #     {'name': 'robot4', 'x': 15.56, 'y': 6.35, 'z': 0.01, 'yaw': -1.57, 'room': 'room_4'},
    # ]

    # In complete_simulation_move.launch.py
    # These are rough estimates - you'll need to fine-tune
    robots = [
        {'name': 'robot1', 'x': 0.0, 'y': 0.0, 'z': 0.01, 'yaw': 1.57, 'room': 'room_1'},   # Upper-left
        # {'name': 'robot2', 'x': -1.2, 'y': 0.0, 'z': 0.01, 'yaw': 3.14, 'room': 'room_2'},   # Upper-right
        # {'name': 'robot3', 'x': -0.2, 'y': 0.8, 'z': 0.0, 'yaw': 1.57, 'room': 'room_3'},    
        # {'name': 'robot3', 'x': -1.2, 'y': 1.0, 'z': 0.01, 'yaw': 0.0, 'room': 'room_3'},  # Lower-right
        {'name': 'robot4', 'x': -0.2, 'y': 0.8, 'z': 0.01, 'yaw': -1.57, 'room': 'room_4'},# Lower-left
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
        # Spawn DIRECTLY at final position (NO temporary position!)
        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{robot["name"]}',
            arguments=[
                '-entity', robot['name'],
                '-file', model_file,
                '-robot_namespace', robot['name'],
                '-x', str(robot['x']),      # Use actual position
                '-y', str(robot['y']),      # Use actual position
                '-z', str(robot['z']),
                '-Y', str(robot['yaw'])     # Use actual yaw
            ],
            output='screen'
        )
        
        # Controller node
        controller = Node(
            package='mazesim',
            executable='robot_controller',
            name='robot_controller',
            namespace=robot['name'],
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot['name'],
                'initial_room': robot['room']
            }]
        )
        
        # Timing: spawn -> wait -> controller
        # Increased delays to avoid collision during spawn
        base_delay = 3.0 + (i * 2.5)
        launch_list.extend([
            TimerAction(period=base_delay, actions=[spawn]),
            TimerAction(period=base_delay + 1.5, actions=[controller])
        ])
    
    return LaunchDescription(launch_list)