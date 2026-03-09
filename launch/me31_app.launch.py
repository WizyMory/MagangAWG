from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Node Backend Modbus
        Node(
            package='me31_modbus',
            executable='modbus_node',
            name='me31_pymodbus_node',
            output='screen'
        ),
        
        # 2. Node UI PyQt5
        Node(
            package='me31_modbus',
            executable='ui_node',
            name='ui_node',
            output='screen'
        )
    ])