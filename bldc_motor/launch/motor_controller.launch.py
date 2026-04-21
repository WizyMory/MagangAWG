import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('bldc_motor'), # Ganti dengan nama package lo
        'config',
        'motor_config.yaml'
    )

    bldc_node = Node(
        package='bldc_motor',          # Ganti dengan nama package lo
        executable='motorController_node', # Nama executable di setup.py
        name='motorController_node',
        output='screen',
        emulate_tty=True,
        parameters=[config]                 # Ini yang masukin file YAML lo otomatis
    )

    return LaunchDescription([
        bldc_node
    ])