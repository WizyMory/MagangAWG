#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from me31_msg.msg import InterfacesData

from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian


class Me31PyModbus(Node):
    def __init__(self):
        super().__init__("me31_pymodbus_node")

        self.client_ = None
        self.slaveID_ = None
        self.baudrate_ = None
        self.currentPort_ = None
        self.parity_ = 'N'
        self.stopbits_ = 1

        self.subscription = self.create_subscription(
            InterfacesData,
            "modbus_command",
            self.callback,
            10
        )

        self.response_pub = self.create_publisher(
            InterfacesData,
            "modbus_response",
            10
        )

        self.reboot_srv = self.create_service(
            Trigger,
            'reboot_slave',
            self.handle_reboot_request
        )

        self.restore_srv = self.create_service(
            Trigger,
            'restore_factory_settings',
            self.handle_restore_request
        )

    # ----------------------------------------------------------
    # ROS2 Pub/Sub Functions
    # ----------------------------------------------------------
    def publish_response(self, slave, address, value, status):
        msg = InterfacesData()
        msg.slave_id = slave
        msg.address = address
        msg.value = int(value)
        msg.mode = status
        self.response_pub.publish(msg)
        
    # ----------------------------------------------------------
    # Modbus Connection Functions
    # ----------------------------------------------------------
    def connect_slave(self, port, baudrate):
        try:
            client = ModbusClient(
                port=port,
                baudrate=baudrate,
                parity=self.parity_,
                stopbits=self.stopbits_
            )

            self.currentPort_ = port
            self.baudrate_ = baudrate
            self.client_ = client

            self.client_.connect()
            self.get_logger().info(f"Connected -> {self.currentPort_} at {self.baudrate_} baud!")
        
        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")
    
    def disconnect(self):
        try:
            if self.client_:
                self.client_.close()
                self.client_ = None
                self.get_logger().info("Disconnected from slave.")
        except Exception as e:
            self.get_logger().error(f"Disconnect error: {e}")

    # ----------------------------------------------------------
    # Modbus Read/Write Functions
    # ----------------------------------------------------------
    def send_data(self, data_type="int", slave_id=1, ui_address=0, value=None):
        """ ui_address: 0=AO1, 1=AO2, 2=AO3, 3=AO4 (Sesuai mapping UI) """
        match data_type:
            case "int":
                if ui_address == 0: self.write_int(slave_id, 0x0064, value)
                if ui_address == 1: self.write_int(slave_id, 0x0065, value)
                if ui_address == 2: self.write_int(slave_id, 0x0066, value)
                if ui_address == 3: self.write_int(slave_id, 0x0067, value)
            case "float":
                if ui_address == 0: self.write_float(slave_id, 0x0000, value)
                if ui_address == 1: self.write_float(slave_id, 0x0002, value)
                if ui_address == 2: self.write_float(slave_id, 0x0004, value)
                if ui_address == 3: self.write_float(slave_id, 0x0006, value)
            case _:
                self.get_logger().error(f"Unsupported data type: {data_type}")
        
    def read_data(self, data_type="int", slave_id=1, ui_address=0):
        """ ui_address: 0=AI1, 1=AI2, 2=AI3, 3=AI4 """
        match data_type:
            case "int":
                if ui_address == 0: return self.read_int(slave_id, 0x0064)
                if ui_address == 1: return self.read_int(slave_id, 0x0065)
                if ui_address == 2: return self.read_int(slave_id, 0x0066)
                if ui_address == 3: return self.read_int(slave_id, 0x0067)
            case "float":
                if ui_address == 0: return self.read_float(slave_id, 0x00C8)
                if ui_address == 1: return self.read_float(slave_id, 0x00CA)
                if ui_address == 2: return self.read_float(slave_id, 0x00CC)
                if ui_address == 3: return self.read_float(slave_id, 0x00CE)
            case _:
                self.get_logger().error(f"Unsupported data type: {data_type}")
                return None

    # ----------------------------------------------------------
    # Write Helper Functions
    # ----------------------------------------------------------
    def write_int(self, slave_id, address, value):
        try:
            self.client_.write_register(address=address, value=value, slave=slave_id)
            self.get_logger().debug(f"Write OK -> Addr:{hex(address)} Val:{value}")
        except Exception as e:
            self.get_logger().error(f"Failed to write register: {e}")
    
    def write_float(self, slave_id, address, value):
        try:
            builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)
            builder.add_32bit_float(value)
            registers = builder.to_registers()
            self.client_.write_registers(address, registers, slave=slave_id)
            self.get_logger().debug(f"Write OK -> Addr:{hex(address)} Val:{value}")
        except Exception as e:
            self.get_logger().error(f"Failed to write float register: {e}")

    # ----------------------------------------------------
    # Read Helper Functions
    # ----------------------------------------------------
    def read_int(self, slave_id, address):
        try:
            result = self.client_.read_input_registers(address=address, count=1, slave=slave_id)
            if result.isError():
                return None
            val = result.registers[0]
            self.get_logger().info(f"Addr:{hex(address)} Val:{val}")
            return val
        except Exception as e:
            self.get_logger().error(f"Read Int Error: {e}")
            return None

    def read_float(self, slave_id, address):
        try:
            result = self.client_.read_input_registers(address=address, count=2, slave=slave_id)
            if result.isError():
                return None
            decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.BIG, wordorder=Endian.BIG)
            value = decoder.decode_32bit_float()
            self.get_logger().info(f"Addr:{hex(address)} Value:{value}")
            return value
        except Exception as e:
            self.get_logger().error(f"Read Float Error: {e}")
            return None
    
    # ----------------------------------------------------
    # Configuration Helper Functions
    # ----------------------------------------------------
    def set_input_range(self, input_range, slave_id):
        if input_range == 0: 
            value = 0x0000
            range_str = "0-20mA"
        elif input_range == 1:
            value = 0x0001
            range_str = "4-20mA"
        else:
            return
        
        try:
            self.write_int(slave_id, 0x04B2, value)
            self.write_int(slave_id, 0x04B3, value)
            self.write_int(slave_id, 0x04B4, value)
            self.write_int(slave_id, 0x04B5, value)
            self.get_logger().info(f"Input range set to {range_str} for slave {slave_id}")
        except Exception as e:
            self.get_logger().error(f"Read Sampling Mode Error: {e}")
    
    def set_output_range(self, output_range, slave_id):
        if output_range == 0: 
            value = 0x0000
            range_str = "0-20mA"
        elif output_range == 1:
            value = 0x0001
            range_str = "4-20mA"
        else:
            return

        try:
            self.write_int(slave_id, 0x0514, value)
            self.write_int(slave_id, 0x0515, value)
            self.write_int(slave_id, 0x0516, value)
            self.write_int(slave_id, 0x0517, value)
            self.get_logger().info(f"Output range set to {range_str} for slave {slave_id}")
        except Exception as e:
            self.get_logger().error(f"Set Output Range Error: {e}")
    
    #----------------------------------------------------
    # Cleanup and Shutdown
    # ----------------------------------------------------
    def reboot_slave(self, slave_id):
        try:
            self.write_int(slave_id=slave_id, address=0x07EA, value=0x5BB5)
            self.get_logger().info(f"Reboot command sent to slave {slave_id}")
        except Exception as e:
            self.get_logger().error(f"Reboot Error: {e}")
    
    def handle_reboot_request(self, request, response):
        try:
            self.reboot_slave(slave_id=1)
            response.success = True
            response.message = "Slave reboot triggered"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def restore_factory_settings(self, slave_id):
        try:
            self.write_int(slave_id=slave_id, address=0x07E9, value=0x5BB5)
            self.get_logger().info(f"Factory reset command sent to slave {slave_id}")
        except Exception as e:
            self.get_logger().error(f"Factory Reset Error: {e}")
    
    def handle_restore_request(self, request, response):
        try:
            self.restore_factory_settings(slave_id=1)
            response.success = True
            response.message = "Factory reset triggered"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def destroy_node(self):
        if self.client_:
            self.client_.close()
            self.get_logger().info("Modbus connection closed.")
        super().destroy_node()

    # ----------------------------------------------------------
    # Subscriber Callback
    # ----------------------------------------------------------
    def callback(self, msg):
        try:
            if msg.mode == "connect":
                self.connect_slave(msg.port, msg.baudrate)
                return 
            
            if msg.mode == "disconnect":
                self.disconnect()
                return
            
            if not self.client_:
                self.get_logger().warning("Not connected to any slave. Ignoring command.")
                return
            
            if msg.mode == "set_input_range":
                self.set_input_range(msg.value, msg.slave_id)
                return
            
            if msg.mode == "set_output_range":
                self.set_output_range(msg.value, msg.slave_id)
                return

            if msg.mode == "write":
                # Asumsi default integer dari UI, jika butuh float bisa diekstrak tipenya
                dataType = "float" if isinstance(msg.value, float) else "int"
                
                self.send_data(
                    data_type=dataType, 
                    slave_id=msg.slave_id, 
                    ui_address=msg.address, 
                    value=msg.value
                )
                return

            if msg.mode == "read":
                result = self.read_data(
                    data_type="int", 
                    slave_id=msg.slave_id, 
                    ui_address=msg.address
                )

                if result is not None:
                    self.publish_response(
                        slave=msg.slave_id,
                        address=msg.address,
                        value=result,
                        status="read_response"
                    )
                return

            if msg.mode == "reboot":
                self.reboot_slave(msg.slave_id)
                return
            
            if msg.mode == "restore":
                self.restore_factory_settings(msg.slave_id)
                return

        except Exception as e:
            self.get_logger().error(f"Callback execution error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Me31PyModbus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()