from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='modbus_devices',
            executable='example_node',
            name='modbus_example',
            parameters=[
                {
                    'port': '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0002T4J-if00-port0',
                    'baudrate': 115200,
                    'slave_id': 1,
                    'register': 0x20A0
                }
            ]
        )
    ])
