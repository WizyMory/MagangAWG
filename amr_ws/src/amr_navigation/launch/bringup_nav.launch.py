from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # === PATH ===
    amr_nav_dir = get_package_share_directory('amr_navigation')
    map_file = os.path.join(amr_nav_dir, 'maps', 'amr_awg_map_best.yaml')
    nav2_param = os.path.join(amr_nav_dir, 'config', 'nav2_params.yaml')
    nav_web_port = LaunchConfiguration('nav_web_port')

    # === HARDWARE (MOTOR + LIDAR + TF) ===
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('amr_motor'),
                'launch',
                'hardware.launch.py'
            )
        )
    )

    # === NAV2 (MAP SERVER + AMCL + CONTROLLER) ===
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_param,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    # === WEB POINT-TO-POINT NAVIGATION UI ===
    nav_web = Node(
        package='amr_nav_interface',
        executable='nav_web_server',
        name='nav_web_server',
        output='screen',
        parameters=[{
            'web_host': '0.0.0.0',
            'web_port': nav_web_port,
            'map_topic': 'map',
            'pose_topic': 'amcl_pose',
            'path_topic': 'plan',
            'initial_pose_topic': 'initialpose',
            'navigate_action': 'navigate_to_pose',
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav_web_port',
            default_value='8020',
            description='AMR navigation web interface port'
        ),
        hardware_launch,
        nav2,
        nav_web
    ])