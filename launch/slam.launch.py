import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_wheelchair = get_package_share_directory('wheelchair')
    
    slam_config_file = PathJoinSubstitution(
        [pkg_wheelchair, 'config', 'slam_toolbox_params.yaml']
    )

    ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wheelchair, 'launch', 'ign_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    slam_nodes = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
          slam_config_file,
          {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        ign_launch,
        slam_nodes
    ])
