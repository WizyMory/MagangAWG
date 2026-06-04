from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    amr_dir = get_package_share_directory('amr_motor')

    slam_params = os.path.join(
        amr_dir, 'config', 'slam_toolbox.yaml'
    )

    # ===============================
    # Include HARDWARE
    # ===============================
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                amr_dir, 'launch', 'hardware.launch.py'
            )
        )
    )

    # ===============================
    # SLAM TOOLBOX (Mapping Mode)
    # ===============================
    slam = TimerAction(
        period=5.0,
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
        hardware,
        slam
    ])
