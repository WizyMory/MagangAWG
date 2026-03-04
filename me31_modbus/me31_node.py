#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
# from me31_interfaces.srv import SendData

from easymodbus import modbusClient 
from easymodbus.modbusClient import (
    convert_float_to_two_registers,
    convert_registers_to_float,
    Parity,
    Stopbits
)

class Me31Modbus(Node):

    def __init__(self):
        super().__init__("me31_node")
        
        # Modbus connection parameters
        self.port = '/dev/ttyUSB0'
        self.baudrate = 9600
        self.mb_conn = self.connect_slave()
        
        # Timer node loop (10 Hz)
        self.create_timer(1/10, self.timer_callback, clock=self.get_clock()) 

        # Services
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

    def connect_slave(self):
        try:    
            client = modbusClient.ModbusClient(self.port)
            
            client.baudrate = self.baudrate
            client.parity = Parity.none
            client.stopbits = Stopbits.one

            client.connect()
            self.get_logger().info(f"Connected to Modbus EBYTE on {self.port}")
            return client
            
        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")
            raise SystemExit

    # ----------------------------------------------------
    # Public API for Data Transmission
    # ----------------------------------------------------
    def send_data(self, data_type="int", slave_id=1, port="", value=None):

        match data_type:
            case "int":
                if port == "AO1": self.write_int(slave_id=slave_id, address=0x0064, value=value)
                if port == "AO2": self.write_int(slave_id=slave_id, address=0x0065, value=value)
                if port == "AO3": self.write_int(slave_id=slave_id, address=0x0066, value=value)
                if port == "AO4": self.write_int(slave_id=slave_id, address=0x0067, value=value)
            case "float":
                if port == "AO1": self.write_float(slave_id=slave_id, address=0x0000, value=value)
                if port == "AO2": self.write_float(slave_id=slave_id, address=0x0002, value=value)
                if port == "AO3": self.write_float(slave_id=slave_id, address=0x0004, value=value)
                if port == "AO4": self.write_float(slave_id=slave_id, address=0x0006, value=value)
            case _:
                self.get_logger().error(f"Unsupported data type: {data_type}")

    def read_data(self):
        pass

    # ----------------------------------------------------
    # Write and Read Helper Functions
    # ----------------------------------------------------
    def write_int(self, slave_id, address, value):
        """Helper untuk menulis register integer."""
        try:
            self.mb_conn.unitidentifier = slave_id
            self.mb_conn.write_single_register(address, value)
            self.get_logger().debug(f"Write OK -> Addr:{hex(address)} Val:{value}")
        except Exception as e:
            self.get_logger().error(f"Write Int Error: {e}")

    def write_float(self, slave_id, address, value):
        """Helper untuk menulis register float (2 registers)."""
        try:
            self.mb_conn.unitidentifier = slave_id
            data = convert_float_to_two_registers(value)

            # Swap byte jika diperlukan oleh slave (umumnya [1, 0])
            self.mb_conn.write_multiple_registers(address, [data[1], data[0]])
        except Exception as e:
            self.get_logger().error(f"Write Float Error: {e}")
    
    def set_output_mode(self, output_mode = 0):
        """Set output mode pada slave."""

        if output_mode == 0: 
            value = 0x0000  # 0-20mA
            mode = "0-20mA"
        elif output_mode == 1:
            value = 0x0001  # 4-20mA
            mode = "4-20mA"
        else:
            self.get_logger().error(f"Invalid output mode: {output_mode}")
            return

        try:
            self.write_int(slave_id=1, address=0x0514, value=value)
            self.write_int(slave_id=1, address=0x0515, value=value)
            self.write_int(slave_id=1, address=0x0516, value=value)
            self.write_int(slave_id=1, address=0x0517, value=value)
            self.get_logger().info(f"Output mode set to {mode}")
        except Exception as e:
            self.get_logger().error(f"Set Output Mode Error: {e}")
    
    #----------------------------------------------------
    # Cleanup and Shutdown
    # ----------------------------------------------------
    def reboot_slave(self, slave_id):
        """Kirim perintah reboot ke slave."""
        
        address = 0x07EA  # Register address for rebooting
        value = 0x5BB5 # Unique value to trigger reboot

        try:
            self.mb_conn.unitidentifier = slave_id
            self.mb_conn.write_single_register(address, value)
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
        """Kirim perintah restore factory settings ke slave."""
        
        address = 0x07E9  # Register address for factory reset
        value = 0x5BB5 # Unique value to trigger factory reset

        try:
            self.mb_conn.unitidentifier = slave_id
            self.mb_conn.write_single_register(address, value)
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
        """Cleanup saat node mati."""
        if hasattr(self, 'mb_conn') and self.mb_conn:
            self.mb_conn.close()
            self.get_logger().info("Modbus connection closed.")
        super().destroy_node()
    
    # ----------------------------------------------------
    # Timer Callback
    # ----------------------------------------------------
    def timer_callback(self):
        """Loop pengiriman data."""
        # Contoh pengiriman data
        self.set_output_mode(output_mode=0) # Set output mode ke 0-20mA

        self.send_data(data_type="float", slave_id=1, port="AO1", value=1.1)
        self.send_data(data_type="float", slave_id=1, port="AO2", value=1.2)
        self.send_data(data_type="float", slave_id=1, port="AO3", value=1.3)
        self.send_data(data_type="float", slave_id=1, port="AO4", value=1.4)

        # self.send_data(data_type="int", slave_id=1, port="AO1", value=5000)
        # self.send_data(data_type="int", slave_id=1, port="AO2", value=5000)
        # self.send_data(data_type="int", slave_id=1, port="AO3", value=5000)
        # self.send_data(data_type="int", slave_id=1, port="AO4", value=5000)


def main(args=None):
    rclpy.init(args=args)
    node = Me31Modbus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()