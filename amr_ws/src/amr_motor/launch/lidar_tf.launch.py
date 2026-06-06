from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.22', '0.165', '0.165',   # x y z (meter)
                '3.1416', '0', '0',         # roll pitch yaw (rad)
                'base_link',
                'laser'
            ],
            name='lidar_tf'
        )
    ])
