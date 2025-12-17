import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_wheelchair = get_package_share_directory('wheelchair')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_wheelchair, 'maps', 'my_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_wheelchair, 'config', 'nav2_params.yaml'))

    # Include Simulation + Robot + EKF + RViz
    ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wheelchair, 'launch', 'ign_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Include Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            # We already launch RViz in ign_launch.py, but Nav2 bringup might try to launch it too if 'use_rviz' is True (default False usually?).
            # Actually, nav2_bringup bringup_launch.py usually doesn't launch RViz unless specified?
            # 'bringup_launch.py' launches localization and navigation.
            # 'tb3_simulation_launch.py' launches rviz.
            # standard 'bringup_launch.py' does NOT launch rviz.
            # However, we are running SLAM? No, this is Navigation mode (AMCL).
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_wheelchair, 'maps', 'my_map.yaml'),
            description='Full path to map yaml file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_wheelchair, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
            
        ign_launch,
        nav2_bringup,
    ])
