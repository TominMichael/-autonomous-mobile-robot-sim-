import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define package names
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_wheelchair = get_package_share_directory('wheelchair')

    # --- File Paths ---
    # Path to the world file
    world_file_path = os.path.join(pkg_wheelchair, 'worlds', 'small_warehouse.sdf')
    # Path to the URDF file
    urdf_file_path = os.path.join(pkg_wheelchair, 'urdf', 'diff_robot.urdf')
    # Path to the RViz config file
    rviz_config_path = os.path.join(pkg_wheelchair, 'urdf', 'rviz.rviz')

    # --- Nodes and Launch Includes ---

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 
                     'robot_description': open(urdf_file_path).read()}]
    )

    # 2. Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'false'
        }.items()
    )

    # 4. Spawn Entity
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'diff_robot'],
        output='screen'
    )

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # --- Launch Description ---
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        rviz_node
    ])
