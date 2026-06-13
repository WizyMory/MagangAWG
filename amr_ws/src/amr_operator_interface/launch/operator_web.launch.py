from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    web_port = LaunchConfiguration('web_port')
    station_file = LaunchConfiguration('station_file')

    operator_web = Node(
        package='amr_operator_interface',
        executable='operator_web_server',
        name='operator_web_server',
        output='screen',
        parameters=[{
            'web_host': '0.0.0.0',
            'web_port': web_port,
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
            'station_file': station_file,
            'pose_timeout': 10.0,
            'station_tolerance_m': 0.35,
            'station_yaw_tolerance_deg': 180.0,
            'battery_empty_voltage': 23.0,
            'battery_full_voltage': 26.2,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'web_port',
            default_value='8030',
            description='AMR operator web interface port',
        ),
        DeclareLaunchArgument(
            'station_file',
            default_value='~/.ros/amr_operator_stations.json',
            description='Persistent station file used by the AMR operator web interface',
        ),
        operator_web,
    ])
