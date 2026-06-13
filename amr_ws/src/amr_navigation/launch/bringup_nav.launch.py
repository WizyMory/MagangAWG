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
    map_file = os.path.join(amr_nav_dir, 'maps', 'amr_awg_lab_new.yaml')
    nav2_param = os.path.join(amr_nav_dir, 'config', 'nav2_params.yaml')
    nav_web_port = LaunchConfiguration('nav_web_port')
    operator_web_port = LaunchConfiguration('operator_web_port')
    operator_station_file = LaunchConfiguration('operator_station_file')

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
            'scan_topic': 'scan',
            'initial_pose_topic': 'initialpose',
            'navigate_action': 'navigate_to_pose',
            'compute_path_action': 'compute_path_to_pose',
            'goal_send_mode': 'topic',
            'cancel_goal_service': '/navigate_to_pose/_action/cancel_goal',
            'motor_enable_service': '/amr_motor/enable',
            'clear_local_costmap_service': '/local_costmap/clear_entirely_local_costmap',
            'clear_global_costmap_service': '/global_costmap/clear_entirely_global_costmap',
            'goal_pose_topic': 'goal_pose',
            'pose_timeout': 10.0,
        }]
    )

    # === WEB OPERATOR STATION UI ===
    operator_web = Node(
        package='amr_operator_interface',
        executable='operator_web_server',
        name='operator_web_server',
        output='screen',
        parameters=[{
            'web_host': '0.0.0.0',
            'web_port': operator_web_port,
            'map_topic': 'map',
            'pose_topic': 'amcl_pose',
            'path_topic': 'plan',
            'scan_topic': 'scan',
            'initial_pose_topic': 'initialpose',
            'goal_pose_topic': 'goal_pose',
            'cmd_vel_topic': '/cmd_vel',
            'navigate_action': 'navigate_to_pose',
            'compute_path_action': 'compute_path_to_pose',
            'goal_send_mode': 'topic',
            'cancel_goal_service': '/navigate_to_pose/_action/cancel_goal',
            'motor_enable_service': '/amr_motor/enable',
            'clear_local_costmap_service': '/local_costmap/clear_entirely_local_costmap',
            'clear_global_costmap_service': '/global_costmap/clear_entirely_global_costmap',
            'battery_voltage_topic': '/amr_motor/bus_voltage',
            'station_file': operator_station_file,
            'pose_timeout': 10.0,
            'station_tolerance_m': 0.35,
            'station_yaw_tolerance_deg': 180.0,
            'battery_empty_voltage': 23.0,
            'battery_full_voltage': 26.2,
        }]
    )

    return LaunchDescription([  
        # DeclareLaunchArgument(
        #     'nav_web_port',
        #     default_value='8020',
        #     description='AMR navigation web interface port'
        # ),
        DeclareLaunchArgument(
            'operator_web_port',
            default_value='8030',
            description='AMR operator station web interface port'
        ),
        DeclareLaunchArgument(
            'operator_station_file',
            default_value='~/.ros/amr_operator_stations.json',
            description='Persistent station list for AMR operator station web interface'
        ),
        hardware_launch,
        nav2,
        # nav_web,
        operator_web
    ])
