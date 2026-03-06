from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(get_package_share_directory('modbus_bridge_pkg'), 'config', 'me31_params.yaml')

    modbus_node = Node(
        package='modbus_bridge_pkg',
            executable='modbus_node',
            name = 'modbus_bridge_node',
            parameters=[config],
            output = 'screen',
            emulate_tty=True
        )

    controller_node = Node(
        package='modbus_bridge_pkg',
            executable='controller',
            parameters=[config],
            output = 'screen',
            emulate_tty=True
        )
    
    gui_node = Node(
        package='modbus_bridge_pkg',
            executable='gui_node',
            parameters=[config],
            output = 'screen',
            emulate_tty=True
        )
    
    return LaunchDescription([
        modbus_node,
        controller_node,
        gui_node
    ])