#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from xaxa0404_msg.msg import DataConnectivity

from easymodbus import modbusClient
from easymodbus.modbusClient import Parity, Stopbits


class ModbusExecutor(Node):

    def __init__(self):
        super().__init__("modbus_executor")

        # subscriber command dari UI
        self.subscription = self.create_subscription(
            DataConnectivity,
            "modbus_command",
            self.callback,
            10
        )

        # publisher response ke UI
        self.response_pub = self.create_publisher(
            DataConnectivity,
            "modbus_response",
            10
        )

        self.client = None
        self.current_port = None

    # ==========================
    # PUBLISH RESPONSE
    # ==========================
    def publish_response(self, slave, address, value, status):

        msg = DataConnectivity()

        msg.slave_id = slave
        msg.address = address
        msg.value = int(value)
        msg.mode = status

        self.response_pub.publish(msg)

    # ==========================
    # CONNECT
    # ==========================
    def connect(self, port, baud):

        try:
            self.client = modbusClient.ModbusClient(port)

            self.client.baudrate = baud
            self.client.parity = Parity.none
            self.client.stopbits = Stopbits.one

            self.client.connect()

            self.current_port = port

            self.get_logger().info(f"Connected -> {port} @ {baud}")

        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")

    # ==========================
    # DISCONNECT
    # ==========================
    def disconnect(self):

        try:
            if self.client:
                self.client.close()
                self.client = None
                self.get_logger().info("Disconnected")

        except Exception as e:
            self.get_logger().error(f"Disconnect error: {e}")

    # ==========================
    # REBOOT
    # ==========================
    def reboot_slave(self, slave):

        try:
            self.client.unitidentifier = slave
            self.client.write_single_register(0x07EA, 0x5BB5)

            self.publish_response(slave, 0x07EA, 1, "reboot_success")

        except Exception as e:
            self.get_logger().error(f"Reboot error: {e}")

    # ==========================
    # RESTORE
    # ==========================
    def restore_factory_settings(self, slave):

        try:
            self.client.unitidentifier = slave
            self.client.write_single_register(0x07E9, 0x5BB5)

            self.publish_response(slave, 0x07E9, 1, "restore_success")

        except Exception as e:
            self.get_logger().error(f"Restore error: {e}")

    # ==========================
    # SET INPUT RANGE
    # ==========================
    def set_input_range(self, slave, value):

        try:

            self.client.unitidentifier = slave

            self.client.write_single_register(0x04B2, value)
            self.client.write_single_register(0x04B3, value)
            self.client.write_single_register(0x04B4, value)
            self.client.write_single_register(0x04B5, value)

            self.publish_response(slave, 0x04B2, value, "input_range_set")

        except Exception as e:
            self.get_logger().error(f"Set input error: {e}")

    # ==========================
    # SET OUTPUT RANGE
    # ==========================
    def set_output_range(self, slave, value):

        try:

            self.client.unitidentifier = slave

            self.client.write_single_register(0x0514, value)
            self.client.write_single_register(0x0515, value)
            self.client.write_single_register(0x0516, value)
            self.client.write_single_register(0x0517, value)

            self.publish_response(slave, 0x0514, value, "output_range_set")

        except Exception as e:
            self.get_logger().error(f"Set output error: {e}")

    # ==========================
    # CALLBACK
    # ==========================
    def callback(self, msg):

        try:

            # ======================
            # CONNECT
            # ======================
            if msg.mode == "connect":
                self.connect(msg.port, msg.baudrate)
                return

            # ======================
            # DISCONNECT
            # ======================
            if msg.mode == "disconnect":
                self.disconnect()
                return

            if not self.client:
                self.get_logger().error("Modbus not connected")
                return

            self.client.unitidentifier = msg.slave_id

            # ======================
            # READ
            # ======================
            if msg.mode == "read":

                if msg.function_code == 3:

                    result = self.client.read_holding_registers(
                        msg.reg_address,
                        msg.quantity
                    )

                elif msg.function_code == 4:

                    result = self.client.read_input_registers(
                        msg.reg_address,
                        msg.quantity
                    )

                else:
                    self.get_logger().warn("Invalid read function")
                    return

                if result:

                    for i, val in enumerate(result):

                        self.publish_response(
                            msg.slave_id,
                            msg.reg_address + i,
                            val,
                            "read_result"
                        )

            # ======================
            # WRITE
            # ======================
            elif msg.mode == "write":

                if msg.function_code == 6:

                    self.client.write_single_register(
                        msg.reg_address,
                        msg.value
                    )

                elif msg.function_code == 16:

                    self.client.write_multiple_registers(
                        msg.reg_address,
                        [msg.value]
                    )

                else:
                    self.get_logger().warn("Invalid write function")
                    return

                self.publish_response(
                    msg.slave_id,
                    msg.reg_address,
                    msg.value,
                    "write_success"
                )

            # ======================
            # REBOOT
            # ======================
            elif msg.mode == "reboot":

                self.reboot_slave(msg.slave_id)

            # ======================
            # RESTORE
            # ======================
            elif msg.mode == "restore":

                self.restore_factory_settings(msg.slave_id)

            # ======================
            # SET INPUT
            # ======================
            elif msg.mode == "set_input_range":

                self.set_input_range(
                    msg.slave_id,
                    msg.value
                )

            # ======================
            # SET OUTPUT
            # ======================
            elif msg.mode == "set_output_range":

                self.set_output_range(
                    msg.slave_id,
                    msg.value
                )

            else:

                self.get_logger().warn(f"Unknown mode: {msg.mode}")

        except Exception as e:

            self.get_logger().error(f"Modbus error: {str(e)}")


def main():

    rclpy.init()

    node = ModbusExecutor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
