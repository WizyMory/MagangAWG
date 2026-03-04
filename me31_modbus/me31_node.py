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
    """
    ROS2 Node untuk komunikasi Modbus dengan EBYTE ME31.
    Fitur:
        - Koneksi serial ke ME31
        - Fungsi baca/tulis register untuk AI/AO
        - Pengaturan sampling range dan output range
        - Service untuk reboot dan restore factory settings
    """

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
    # Public API for Data Read/Write
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
    
    def read_data(self, data_type="int", slave_id=1, port=""):

        match data_type:
            case "int":
                if port == "AI1": return self.read_int(slave_id=slave_id, address=0x0064)
                if port == "AI2": return self.read_int(slave_id=slave_id, address=0x0065)
                if port == "AI3": return self.read_int(slave_id=slave_id, address=0x0066)
                if port == "AI4": return self.read_int(slave_id=slave_id, address=0x0067)
            case "float":
                if port == "AI1": return self.read_float(slave_id=slave_id, address=0x00C8)
                if port == "AI2": return self.read_float(slave_id=slave_id, address=0x00CA)
                if port == "AI3": return self.read_float(slave_id=slave_id, address=0x00CC)
                if port == "AI4": return self.read_float(slave_id=slave_id, address=0x00CE)
            case _:
                self.get_logger().error(f"Unsupported data type: {data_type}")
                return None

    # ----------------------------------------------------
    # Read Helper Functions
    # ----------------------------------------------------
    def read_int(self, slave_id, address):
        """Helper untuk membaca register integer."""
        try:
            self.mb_conn.unitidentifier = slave_id
            result = self.mb_conn.read_inputregisters(address, 1)
            return result[0]
        except Exception as e:
            self.get_logger().error(f"Read Int Error: {e}")
            return None
    
    def read_float(self, slave_id, address):
        """Helper untuk membaca register float (2 registers)."""
        try:
            self.mb_conn.unitidentifier = slave_id
            result = self.mb_conn.read_inputregisters(address, 2)
            # Swap byte jika diperlukan oleh slave (umumnya [1, 0])
            return convert_registers_to_float([result[1], result[0]])
        except Exception as e:
            self.get_logger().error(f"Read Float Error: {e}")
            return None

    # ----------------------------------------------------
    # Write Helper Functions
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
    
    # ----------------------------------------------------
    # Configuration Helper Functions
    # ----------------------------------------------------
    def set_input_range(self, input_range, slave_id):
        """Helper untuk mengatur sampling range pada slave."""

        if input_range == 0: 
            value = 0x0000  # 0-20mA
            range = "0-20mA"
        elif input_range == 1:
            value = 0x0001  # 4-20mA
            range = "4-20mA"
        else:
            self.get_logger().error(f"Invalid input range: {input_range}")
            return
        
        try:
            self.write_int(slave_id=slave_id, address=0x04B2, value=value)
            self.write_int(slave_id=slave_id, address=0x04B3, value=value)
            self.write_int(slave_id=slave_id, address=0x04B4, value=value)
            self.write_int(slave_id=slave_id, address=0x04B5, value=value)
            self.get_logger().info(f"Input range set to {range} for slave {slave_id}")
        except Exception as e:
            self.get_logger().error(f"Read Sampling Mode Error: {e}")
            return None
    
    def set_output_range(self, output_range, slave_id):
        """Set output range pada slave."""

        if output_range == 0: 
            value = 0x0000  # 0-20mA
            range = "0-20mA"
        elif output_range == 1:
            value = 0x0001  # 4-20mA
            range = "4-20mA"
        else:
            self.get_logger().error(f"Invalid output range: {output_range}")
            return

        try:
            self.write_int(slave_id=slave_id, address=0x0514, value=value)
            self.write_int(slave_id=slave_id, address=0x0515, value=value)
            self.write_int(slave_id=slave_id, address=0x0516, value=value)
            self.write_int(slave_id=slave_id, address=0x0517, value=value)
            self.get_logger().info(f"Output range set to {range} for slave {slave_id}")
        except Exception as e:
            self.get_logger().error(f"Set Output Range Error: {e}")

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
        """Service handler untuk reboot slave."""
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
        """Service handler untuk restore factory settings."""
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
        # Set sampling range dan output range (opsional, diatur sekali saat startup)
        self.set_output_range(output_range=0, slave_id=1)   # Set output range ke 0-20mA
        self.set_input_range(input_range=0, slave_id=1)     # Set input range ke 0-20mA

        # Contoh pengiriman data (nanti dijadiin pub sub)
        self.send_data(data_type="int", slave_id=1, port="AO1", value=3)
        self.send_data(data_type="int", slave_id=1, port="AO2", value=4)
        self.send_data(data_type="int", slave_id=1, port="AO3", value=5)
        self.send_data(data_type="int", slave_id=1, port="AO4", value=6)

        self.send_data(data_type="float", slave_id=1, port="AO1", value=1.1)
        self.send_data(data_type="float", slave_id=1, port="AO2", value=1.2)
        self.send_data(data_type="float", slave_id=1, port="AO3", value=1.3)
        self.send_data(data_type="float", slave_id=1, port="AO4", value=1.4)

        # Contoh pembacaan data (nanti dijadiin pub sub)
        self.read_data(data_type="int", slave_id=1, port="AI1")
        self.read_data(data_type="int", slave_id=1, port="AI2")
        self.read_data(data_type="int", slave_id=1, port="AI3")
        self.read_data(data_type="int", slave_id=1, port="AI4")

        self.read_data(data_type="float", slave_id=1, port="AI1")
        self.read_data(data_type="float", slave_id=1, port="AI2")
        self.read_data(data_type="float", slave_id=1, port="AI3")
        self.read_data(data_type="float", slave_id=1, port="AI4")


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