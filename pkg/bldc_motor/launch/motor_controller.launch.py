import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('bldc_motor'), 
        'config',
        'motor_config.yaml'
    )

    bldc_node = Node(
        package='bldc_motor',          
        executable='motorController_node', 
        name='motorController_node',
        output='screen',
        emulate_tty=True,
        parameters=[config]               
    )

    return LaunchDescription([
        bldc_node
    ])