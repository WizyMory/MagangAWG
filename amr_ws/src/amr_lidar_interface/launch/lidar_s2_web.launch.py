from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    sllidar_launch = os.path.join(
        sllidar_dir,
        'launch',
        'sllidar_s2_launch.py',
    )

    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')
    scan_topic = LaunchConfiguration('scan_topic')
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    max_points = LaunchConfiguration('max_points')
    disconnect_timeout = LaunchConfiguration('disconnect_timeout')
    log_dir = LaunchConfiguration('log_dir')
    log_history_limit = LaunchConfiguration('log_history_limit')
    running_log_interval = LaunchConfiguration('running_log_interval')

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch),
        launch_arguments={
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode,
        }.items(),
    )

    web_bridge = Node(
        package='amr_lidar_interface',
        executable='lidar_web_server',
        name='lidar_web_server',
        output='screen',
        parameters=[{
            'scan_topic': scan_topic,
            'web_host': web_host,
            'web_port': web_port,
            'max_points': max_points,
            'disconnect_timeout': disconnect_timeout,
            'log_dir': log_dir,
            'log_history_limit': log_history_limit,
            'running_log_interval': running_log_interval,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value='serial',
            description='SLLidar channel type',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_80a0d274a582eb11b24e988d9693f7bc-if00-port0',
            description='SLLidar S2 serial port',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='1000000',
            description='SLLidar S2 serial baudrate',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='LaserScan frame_id',
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Invert scan data',
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Enable angle compensation',
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='DenseBoost',
            description='SLLidar scan mode',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan',
            description='LaserScan topic consumed by the web bridge',
        ),
        DeclareLaunchArgument(
            'web_host',
            default_value='0.0.0.0',
            description='FastAPI bind host',
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8010',
            description='FastAPI bind port',
        ),
        DeclareLaunchArgument(
            'max_points',
            default_value='720',
            description='Maximum scan points sent to the browser',
        ),
        DeclareLaunchArgument(
            'disconnect_timeout',
            default_value='3.0',
            description='Seconds without LaserScan before warning/disconnected status',
        ),
        DeclareLaunchArgument(
            'log_dir',
            default_value=os.path.expanduser('~/amr_ws/src/amr_lidar_interface/logs'),
            description='Directory for lidar troubleshooting CSV logs',
        ),
        DeclareLaunchArgument(
            'log_history_limit',
            default_value='300',
            description='Maximum lidar log events kept in the web timeline buffer',
        ),
        DeclareLaunchArgument(
            'running_log_interval',
            default_value='30.0',
            description='Seconds between running heartbeat log entries',
        ),
        lidar_driver,
        web_bridge,
    ])
