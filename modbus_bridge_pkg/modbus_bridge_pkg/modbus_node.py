from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pymodbus.client import ModbusSerialClient as ModbusClient
from std_srvs.srv import SetBool, Trigger
from .modbus_func import ME31Handler
import time

class ModbusBridgeNode(Node):
    def __init__(self):
        super().__init__('modbus_bridge_node')

        # PARAMETER DEFAULT
        self.declare_parameters(
            namespace= '',
            parameters = [
                ('port', '/dev/ttyUSB0'),
                ('slave_id', 1),
                ('baudrate', 9600),
                ('timeout', 1),
                ('parity', 'N'),
                ('ai_factor', 0.001),
                ('ao_factor', 1000.0),
                ('reboot_address', 0x07EA),  # 2026 desimalea
                ('reboot_value', 0x5BB5),
                ('mode_address', 0x0514) 
                ])

        # DEFAULT ADDRESS
        self.declare_parameters(
            namespace = '',
            parameters=[
                ('ao1_address', 100),
                ('ao2_address', 101),
                ('ao3_address', 202),
                ('ao4_address', 203),
                ('ai1_address', 100),
                ('ai2_address', 101),
                ('ai3_address', 102),
                ('ai4_address', 103)])

        # GET PARAMETER
        port = self.get_parameter('port').value
        self.slave_id = self.get_parameter('slave_id').value
        baud = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        parity = self.get_parameter('parity').value

        self.ai_factor = self.get_parameter('ai_factor').value
        self.ao_factor = self.get_parameter('ao_factor').value
        self.addr_reboot = self.get_parameter('reboot_address').value
        self.reboot_val = self.get_parameter('reboot_value').value
        self.addr_mode = self.get_parameter('mode_address').value

        self.addr_ao = [
            self.get_parameter('ao1_address').value,
            self.get_parameter('ao2_address').value,
            self.get_parameter('ao3_address').value,
            self.get_parameter('ao4_address').value
        ]
        
        self.addr_ai = [
            self.get_parameter('ai1_address').value,
            self.get_parameter('ai2_address').value,
            self.get_parameter('ai3_address').value,
            self.get_parameter('ai4_address').value
        ]

        # SETUP CLIENT & HANDLER
        self.client = ModbusClient(port=port, baudrate=baud, timeout=timeout, parity=parity, stopbits=1, bytesize=8) 
        self.func=ME31Handler(self.client, self.get_logger())
        
        try:
            self.client.connect()
            self.get_logger().info(f"Berhasil terhubung ke Modbus di {port} dengan baudrate {baud}")
        except Exception as e:
            self.get_logger().error(f"Gagal terhubung ke Modbus di {port}: {e}")

        # PUBLISHER
        self.pub = self.create_publisher(Float32, "/AnalogInput1", 10)

        # SUBSCRIBER
        self.subs = []
        for i in range(4):
            topic_name = f"/AO{i+1}"
            sub = self.create_subscription(
                Float32, 
                topic_name, 
                partial(self.ao_callback, addr=self.addr_ao[i], chi = i+1),
                10
            )
            self.subs.append(sub)

        # TIMER
        self.create_timer(10.0, self.read_loop)
        self.create_timer(1.0, self.watchdog_timer_callback)

        # SERVICE
        self.mode_service = self.create_service(SetBool, 'set_ao_mode', self.setMode_callback)
        self.reboot_service = self.create_service(Trigger, 'reboot_device', self.reboot_callback)
        
        self.last_ao_time = self.get_clock().now()
        self.watchdog_timeout = 5.0
        self.first_msg_received = False
    
    def read_loop(self):
        res = self.func.read_ai(self.addr_ai[0], count=4, id=self.slave_id)
        if res is not None:
            log_msg = "Reading Input..."
            for i in range(4):
                msg = Float32()
                msg.data = res[i] * self.ai_factor
                self.pub.publish(msg)
                    
                log_msg += f"| AI{i+1}: {msg.data:.3f} "
            self.get_logger().info(log_msg)

    def ao_callback(self, msg, addr, chi):
        self.get_logger().info(f"Received AO{chi} value: {msg.data:.3f}")
        self.first_msg_received = True
        self.last_ao_time = self.get_clock().now()
        factor = self.ao_factor

        val_ua = int(msg.data * factor)
        return self.func.write_multiple(addr, val_ua)

    def watchdog_timer_callback(self):
        if not self.first_msg_received:
            return
        
        now = self.get_clock().now()
        elapsed = (now - self.last_ao_time).nanoseconds / 1e9

        if elapsed > self.watchdog_timeout:
            self.get_logger().warn(f"Watchdog timeout! Tidak ada update AO selama {elapsed:.2f} detik.")
            # Di sini kamu bisa reset AO ke nilai default atau lakukan tindakan lain sesuai kebutuhan
            self.apply_safety_state()
    
    def apply_safety_state(self):
        # Contoh: reset semua AO ke 0
        for i in range(4):
            self.func.write_single(self.addr_ao[i], 0)

    def setMode_callback(self, request, response):
        mode_val = 1 if request.data else 0 
        
        try:
            res = self.func.write_single(self.addr_mode, mode_val)
            
            if res:
                response.success = True
                response.message = f"Mode AO berhasil diubah ke {'4-20mA' if request.data else '0-20mA'}"
                self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Error Python: {str(e)}"
            self.get_logger().error(response.message)
            
        return response
    
    def reboot_callback(self, request, response):        
        self.get_logger().info(f"Try to reboot device...")
        
        try:
            res = self.func.write_single(self.addr_reboot, self.reboot_val)
            if not res:
                response.success = False
                response.message = "Failed to reboot device"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        return response
    

def main(args=None):
    # Memulai sistem ROS 2
    rclpy.init(args=args)
    
    # Membuat instance dari class Node kamu
    node = ModbusBridgeNode()
    
    try:
        # Menjalankan node agar terus memantau Modbus
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Biar rapi pas di-stop (Ctrl+C)
        node.get_logger().info('Node dimatikan oleh user.')
    finally:
        # Bersihkan koneksi Modbus dan matikan ROS
        node.destroy_node()
        rclpy.shutdown()

# Bagian ini penting biar script bisa dijalankan langsung
if __name__ == '__main__':
    main()