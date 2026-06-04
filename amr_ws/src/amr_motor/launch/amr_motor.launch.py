from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_motor',
            executable='amr_motor_node',
            parameters=['config/motor.yaml']
        )
    ])
