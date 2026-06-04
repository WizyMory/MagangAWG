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
        lidar_driver,
        web_bridge,
    ])
