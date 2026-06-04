import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

import threading
import time
import signal
import sys

from modbus_devices.modbus_client import ModbusRTUClient


class ExampleNode(Node):
    def __init__(self):
        super().__init__('modbus_example')

        # ===================== PARAMETERS =====================
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('slave_id', 1)

        # registers (0-based)
        self.declare_parameter('reg_bus_voltage', 0x20A1)     # 20A1h
        self.declare_parameter('reg_motor_temp', 0x20A4)      # 20A4h
        self.declare_parameter('reg_driver_temp', 0x20B0)     # 20B0h

        self.declare_parameter('voltage_scale', 0.01)        # calibrated
        self.declare_parameter('driver_temp_scale', 0.1)      # datasheet
        self.declare_parameter('poll_period', 1.0)
        self.declare_parameter('data_timeout', 2.0)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.slave_id = self.get_parameter('slave_id').value

        self.reg_bus_voltage = self.get_parameter('reg_bus_voltage').value
        self.reg_motor_temp = self.get_parameter('reg_motor_temp').value
        self.reg_driver_temp = self.get_parameter('reg_driver_temp').value

        self.voltage_scale = self.get_parameter('voltage_scale').value
        self.driver_temp_scale = self.get_parameter('driver_temp_scale').value

        self.poll_period = self.get_parameter('poll_period').value
        self.data_timeout = self.get_parameter('data_timeout').value

        self.get_logger().info(
            f'Using port={port}, baudrate={baudrate}'
        )

        # ===================== MODBUS =====================
        self.modbus = ModbusRTUClient(port=port, baudrate=baudrate)

        # ===================== DATA BUFFER =====================
        self.bus_voltage = None
        self.motor_temp_left = None
        self.motor_temp_right = None
        self.driver_temp = None
        self.last_update_time = None
        self.running = True

        self.last_warn_time = 0.0
        self.warn_interval = 5.0

        # ===================== THREAD =====================
        self.worker = threading.Thread(
            target=self.modbus_loop,
            daemon=True
        )
        self.worker.start()

        # ===================== ROS =====================
        self.pub_voltage = self.create_publisher(Float32, 'sensor/voltage', 10)
        self.pub_motor_left = self.create_publisher(Float32, 'sensor/motor_temp_left', 10)
        self.pub_motor_right = self.create_publisher(Float32, 'sensor/motor_temp_right', 10)
        self.pub_driver = self.create_publisher(Float32, 'sensor/driver_temp', 10)
        self.pub_status = self.create_publisher(Bool, 'sensor/connected', 10)

        self.timer = self.create_timer(0.5, self.publish_loop)

    # ==========================================================
    def modbus_loop(self):
        while self.running:
            try:
                # Bus Voltage
                v = self.modbus.read_holding_registers(
                    self.slave_id, self.reg_bus_voltage, 1
                )

                # Motor temperature (1 register, 2 motor)
                mt = self.modbus.read_holding_registers(
                    self.slave_id, self.reg_motor_temp, 1
                )

                # Driver temperature
                dt = self.modbus.read_holding_registers(
                    self.slave_id, self.reg_driver_temp, 1
                )

                if v and mt and dt:
                    self.bus_voltage = round(v[0] * self.voltage_scale, 2)

                    # motor temp split high / low byte
                    raw = mt[0]
                    self.motor_temp_left = float((raw >> 8) & 0xFF)
                    self.motor_temp_right = float(raw & 0xFF)

                    self.driver_temp = round(dt[0] * self.driver_temp_scale, 1)

                    self.last_update_time = time.time()
                else:
                    self.bus_voltage = None

            except Exception as e:
                self.get_logger().error(str(e))
                self.bus_voltage = None

            time.sleep(self.poll_period)

    # ==========================================================
    def publish_loop(self):
        now = time.time()

        connected = (
            self.bus_voltage is not None
            and self.last_update_time is not None
            and (now - self.last_update_time) <= self.data_timeout
        )

        status = Bool()
        status.data = connected
        self.pub_status.publish(status)

        if not connected:
            if now - self.last_warn_time > self.warn_interval:
                self.get_logger().warn('Modbus device not responding')
                self.last_warn_time = now
            return

        self.pub_voltage.publish(Float32(data=self.bus_voltage))
        self.pub_motor_left.publish(Float32(data=self.motor_temp_left))
        self.pub_motor_right.publish(Float32(data=self.motor_temp_right))
        self.pub_driver.publish(Float32(data=self.driver_temp))

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = ExampleNode()

    def signal_handler(sig, frame):
        node.get_logger().info('Shutting down cleanly...')
        node.running = False
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
