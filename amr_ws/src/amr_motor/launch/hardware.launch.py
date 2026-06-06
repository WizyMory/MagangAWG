from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ===============================
    # Paths
    # ===============================
    sllidar_dir = get_package_share_directory('sllidar_ros2')

    lidar_launch = os.path.join(
        sllidar_dir, 'launch', 'sllidar_s2_launch.py'
    )

    # ===============================
    # Nodes (HARDWARE ONLY)
    # ===============================

    # Static TF: base_link -> laser
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=[
            '0.22', '0.165', '0.165',
            '3.1416', '0', '0',
            'base_link', 'laser'
        ]
    )

    # Motor driver (odom publisher)
    amr_motor = Node(
        package='amr_motor',
        executable='move_motor',
        output='screen'
    )

    # LiDAR driver
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch)
    )

    return LaunchDescription([
        static_tf,
        amr_motor,
        lidar
    ])
