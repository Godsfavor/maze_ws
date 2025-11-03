import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import tempfile

def generate_launch_description():
    # Get package directories
    mazesim_dir = get_package_share_directory('mazesim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Paths
    world_file = os.path.join(mazesim_dir, 'worlds', 'maze_world.world')
    urdf_file = os.path.join(mazesim_dir, 'urdf', 'turtlebot3_burger_fixed.urdf')
    
    # Set Gazebo model path
    gazebo_model_path = os.path.join(mazesim_dir, 'models')
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc_template = infp.read()
    
    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot configurations
    robots = [
        {'name': 'robot1', 'x': 3.81, 'y': 16.10, 'z': 0.5, 'yaw': 0.0, 'room': 'room_1'},
        {'name': 'robot2', 'x': 16.31, 'y': 16.10, 'z': 0.5, 'yaw': 0.0, 'room': 'room_2'},
        {'name': 'robot3', 'x': 3.81, 'y': 3.60, 'z': 0.5, 'yaw': 0.0, 'room': 'room_3'},
        {'name': 'robot4', 'x': 16.31, 'y': 3.60, 'z': 0.5, 'yaw': 0.0, 'room': 'room_4'},
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        gzserver,
        gzclient
    ]
    
    # Create temporary URDF files for each robot
    temp_urdf_files = []
    
    # Add each robot
    for i, robot in enumerate(robots):
        robot_name = robot['name']
        
        # Create namespaced URDF by replacing link names
        robot_desc = robot_desc_template
        robot_desc = robot_desc.replace('base_footprint', f'{robot_name}/base_footprint')
        robot_desc = robot_desc.replace('base_link', f'{robot_name}/base_link')
        robot_desc = robot_desc.replace('base_scan', f'{robot_name}/base_scan')
        robot_desc = robot_desc.replace('wheel_left_link', f'{robot_name}/wheel_left_link')
        robot_desc = robot_desc.replace('wheel_right_link', f'{robot_name}/wheel_right_link')
        robot_desc = robot_desc.replace('caster_back_link', f'{robot_name}/caster_back_link')
        robot_desc = robot_desc.replace('imu_link', f'{robot_name}/imu_link')
        robot_desc = robot_desc.replace('wheel_left_joint', f'{robot_name}/wheel_left_joint')
        robot_desc = robot_desc.replace('wheel_right_joint', f'{robot_name}/wheel_right_joint')
        robot_desc = robot_desc.replace('caster_back_joint', f'{robot_name}/caster_back_joint')
        
        # Create temporary file for this robot's URDF
        temp_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
        temp_file.write(robot_desc)
        temp_file.close()
        temp_urdf_files.append(temp_file.name)
        
        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc
            }]
        )
        
        # Spawn robot using -file instead of -topic
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{robot_name}',
            output='screen',
            arguments=[
                '-entity', robot_name,
                '-file', temp_file.name,
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', str(robot['z']),
                '-Y', str(robot['yaw'])
            ]
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
        
        # Calculate delays
        base_delay = 3.0 + (i * 2.5)
        
        launch_description_list.extend([
            TimerAction(period=base_delay, actions=[robot_state_publisher]),
            TimerAction(period=base_delay + 1.0, actions=[spawn_robot]),
            TimerAction(period=base_delay + 2.0, actions=[robot_controller])
        ])
    
    return LaunchDescription(launch_description_list)