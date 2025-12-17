import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- File Paths ---
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_wheelchair = get_package_share_directory('wheelchair') # Replace with your package name

    urdf_file = os.path.join(pkg_wheelchair, 'urdf', 'ign_robot.urdf') # Use the Ignition-compatible URDF
    rviz_config_file = os.path.join(pkg_wheelchair, 'urdf', 'rviz.rviz')
    controller_config_file = os.path.join(pkg_wheelchair, 'config', 'diff_drive.yaml')
    ekf_config_file = os.path.join(pkg_wheelchair, 'config', 'ekf.yaml')
    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Nodes and Launch Includes ---

    # 1. Gazebo Sim
    # Starts Ignition Gazebo with an empty world.
    world_file = os.path.join(pkg_wheelchair, 'worlds', 'maze.sdf')

    # 1. Gazebo Sim
    # Starts Ignition Gazebo with the maze world.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Robot State Publisher
    # Reads the URDF and publishes /robot_description and TF transforms.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file).read()
        }]
    )



    # 3. Spawn Robot
    # Spawns the robot into Gazebo from the /robot_description topic.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diff_robot',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odom_fused')]
    )

    # 4. ROS Gz Bridge
    # Creates a bridge between ROS 2 topics and Gazebo topics for sensors.
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_wheelchair, 'config', 'bridge.yaml'),  # Updated path
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./scan.publisher.reliability': 'reliable',
            'qos_overrides./odom.publisher.reliability': 'reliable',
            'qos_overrides./imu.publisher.reliability': 'reliable',
        }],
        output='screen'
    )
    
    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        
        gz_sim,
        bridge_node,
        robot_state_publisher_node,
        spawn_node,
        rviz_node,
        robot_localization_node,
    ])

