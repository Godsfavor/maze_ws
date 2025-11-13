import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # Get package directories
    mazesim_dir = get_package_share_directory('mazesim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Paths
    world_file = os.path.join(mazesim_dir, 'worlds', 'maze_world.world')
    
    # Use TurtleBot3 model from gazebo package (includes proper SDF)
    model_folder = os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_burger')
    
    # Set Gazebo model paths
    gazebo_model_path = os.path.join(mazesim_dir, 'models')
    turtlebot3_model_path = os.path.join(turtlebot3_gazebo_dir, 'models')
    
    os.environ['GAZEBO_MODEL_PATH'] = f"{gazebo_model_path}:{turtlebot3_model_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"
    
    # Robot configurations
    # robots = [
    #     {'name': 'robot1', 'x': 3.81, 'y': 16.10, 'z': 0.2, 'yaw': 0.0, 'room': 'room_1'},
    #     {'name': 'robot2', 'x': 16.31, 'y': 16.10, 'z': 0.2, 'yaw': 0.0, 'room': 'room_2'},
    #     {'name': 'robot3', 'x': 3.81, 'y': 3.60, 'z': 0.2, 'yaw': 0.0, 'room': 'room_3'},
    #     {'name': 'robot4', 'x': 16.31, 'y': 3.60, 'z': 0.2, 'yaw': 0.0, 'room': 'room_4'},
    # ]

    robots = [
        {'name': 'robot1', 'x': 0.0, 'y': 0.0, 'z': 0.01, 'yaw': 1.57, 'room': 'room_1'},   # Upper-left
        {'name': 'robot2', 'x': -1.2, 'y': 0.0, 'z': 0.01, 'yaw': 3.14, 'room': 'room_2'},   # Upper-right
        # {'name': 'robot3', 'x': -0.2, 'y': 0.8, 'z': 0.0, 'yaw': 1.57, 'room': 'room_3'},    
        {'name': 'robot3', 'x': -1.2, 'y': 1.0, 'z': 0.01, 'yaw': 0.0, 'room': 'room_3'},  # Lower-right
        {'name': 'robot4', 'x': -0.2, 'y': 0.8, 'z': 0.01, 'yaw': -1.57, 'room': 'room_4'},# Lower-left
    ]
    
    # Start Gazebo
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )
    
    launch_description_list = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gzserver,
        gzclient
    ]
    
    # Add each robot using direct model spawning
    for i, robot in enumerate(robots):
        robot_name = robot['name']
        
        # Use ExecuteProcess with gz command for reliable positioning
        spawn_cmd = ExecuteProcess(
            cmd=[
                'gz', 'model',
                '--spawn-file', os.path.join(model_folder, 'model.sdf'),
                '--model-name', robot_name,
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', str(robot['z']),
                '-Y', str(robot['yaw'])
            ],
            output='screen'
        )
        
        # Robot controller node
        robot_controller = Node(
            package='mazesim',
            executable='robot_controller',
            name='robot_controller',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot_name,
                'initial_room': robot['room']
            }]
        )
        
        # Spawn with delays
        base_delay = 3.0 + (i * 2.0)
        launch_description_list.append(
            TimerAction(period=base_delay, actions=[spawn_cmd])
        )
        launch_description_list.append(
            TimerAction(period=base_delay + 1.0, actions=[robot_controller])
        )
    
    return LaunchDescription(launch_description_list)