from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ===============================
    # Paths
    # ===============================
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    amr_dir = get_package_share_directory('amr_motor')

    lidar_launch = os.path.join(
        sllidar_dir, 'launch', 'sllidar_s2_launch.py'
    )

    slam_params = os.path.join(
        amr_dir, 'config', 'slam_toolbox.yaml'
    )

    # ===============================
    # Nodes
    # ===============================

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

    amr_motor = Node(
        package='amr_motor',
        executable='move_motor',
        output='screen'
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch)
    )

    slam = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params, {'use_sim_time': False}]
            )
        ]
    )

    return LaunchDescription([
        static_tf,
        amr_motor,
        lidar,      # ✅ pakai launch lidar
        slam
    ])
