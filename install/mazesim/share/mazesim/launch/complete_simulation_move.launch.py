import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument
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
    
    robots = [
        {'name': 'robot1', 'x': 4.06, 'y': 14.85, 'z': 0.0, 'yaw': 1.57, 'room': 'room_1'},
        {'name': 'robot2', 'x': 15.56, 'y': 14.85, 'z': 0.0, 'yaw': 3.14, 'room': 'room_2'},
        # {'name': 'robot3', 'x': 4.06, 'y': 3.85, 'z': 0.0, 'yaw': 0.0, 'room': 'room_3'},
        # {'name': 'robot4', 'x': 15.56, 'y': 6.35, 'z': 0.0, 'yaw': -1.57, 'room': 'room_4'},
    
    ]
    
    # Gazebo launch (unchanged, assuming gazebo_ros is for Classic)
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
        # Temporary spawn positions to avoid initial collisions
        temp_x = i * 2.0
        temp_y = i * 2.0
        
        # Spawn using spawn_entity.py (compatible with Classic)
        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{robot["name"]}',
            arguments=[
                '-entity', robot['name'],
                '-file', model_file,
                '-robot_namespace', robot['name'],
                '-x', str(temp_x),
                '-y', str(temp_y),
                '-z', str(robot['z']),
                '-R', '0.0',
                '-P', '0.0',
                '-Y', str(robot['yaw'])
            ],
            output='screen'
        )
        
        # Move to final position using /gazebo/set_model_state service
        move_cmd = [
            'ros2', 'service', 'call',
            '/gazebo/set_model_state',
            'gazebo_msgs/srv/SetModelState',
            '{model_state: {model_name: "' + robot['name'] + '", '
            'pose: {position: {x: ' + str(robot['x']) + ', y: ' + str(robot['y']) + ', z: ' + str(robot['z']) + '}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
        ]
        move = ExecuteProcess(
            cmd=move_cmd,
            output='screen'
        )
        
        # Controller node (unchanged)
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
        
        # Timing to ensure sequential execution (spawn -> move -> controller)
        base_delay = 3.0 + (i * 3.0)
        launch_list.extend([
            TimerAction(period=base_delay, actions=[spawn]),
            TimerAction(period=base_delay + 1.0, actions=[move]),
            TimerAction(period=base_delay + 2.0, actions=[controller])
        ])
    
    return LaunchDescription(launch_list)