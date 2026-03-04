from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pymodbus.client import ModbusSerialClient as ModbusClient
from std_srvs.srv import SetBool, Trigger

class ModbusBridgeNode(Node):
    def __init__(self):
        super().__init__('modbus_bridge_node')

        self.last_ao_time = self.get_clock().now()
        self.watchdog_timeout = 5.0
        self.first_msg_received = False

        self.reboot_service = self.create_service(Trigger, 'reboot_device', self.reboot_callback)

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 1)
        self.declare_parameter('parity', 'N')   
        self.declare_parameter('ai_address', 100)
        self.declare_parameter('ai_factor', 0.001)

        # ANALOG OUTPUT
        self.declare_parameter('ao1_address', 1)
        self.declare_parameter('ao2_address', 101)
        self.declare_parameter('ao3_address', 1301)
        self.declare_parameter('ao4_address', 201)

        self.declare_parameter('ao_factor', 1000.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        parity = self.get_parameter('parity').value
        self.client = ModbusClient(port=port, baudrate=baud, timeout=timeout, parity=parity, stopbits=1, bytesize=8) 

        AO1_add = self.get_parameter('ao1_address').value
        AO2_add = self.get_parameter('ao2_address').value
        AO3_add = self.get_parameter('ao3_address').value
        AO4_add = self.get_parameter('ao4_address').value

        if self.client.connect():
            self.get_logger().info(f"Berhasil terhubung ke Modbus di {port} dengan baudrate {baud}")
        else:
            self.get_logger().error(f"Gagal terhubung ke Modbus di {port} dengan baudrate {baud}")

        self.subs = []
        addr = [AO1_add, AO2_add, AO3_add, AO4_add]

        for i in range(4):
            topic_name = f"/AO{i+1}"
            address = addr[i]
            sub = self.create_subscription(
                Float32, 
                topic_name, 
                partial(self.ao_callback, addr=address, chi = i+1),
                10
            )

            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to {topic_name} for AO{i+1} at Modbus address {address}")
            
        self.pub = self.create_publisher(Float32, "/AnalogInput1", 10)
        self.mode_service = self.create_service(SetBool, 'set_ao_mode', self.setMode_callback)
        self.create_timer(10.0, self.loop_baca)
        self.create_timer(1.0, self.watchdog_timer_callback)
        self.read_service = self.create_service(Trigger, 'read_ai', self.read_ai_callback)

    def loop_baca(self):
        addr = self.get_parameter('ai_address').value
        factor = self.get_parameter('ai_factor').value
        
        res = self.client.read_input_registers(addr, count = 4, device_id = 1)
        if not res.isError():
            log_msg = "Reading Input..."
            for i in range(4):
                msg = Float32()
                msg.data = res.registers[i] * factor
                self.pub.publish(msg)
                
                log_msg += f"| AI{i+1}: {msg.data:.3f} "
            
            self.get_logger().info(log_msg)
        else:
            self.get_logger().error(f"Error membaca register AI di address {addr}")

    def read_ai_callback(self, request, response):
        addr = self.get_parameter('ai_address').value
        factor = self.get_parameter('ai_factor').value
        
        res = self.client.read_input_registers(addr, count = 4, device_id = 1)
        if not res.isError():
            values = []
            for i in range(4):
                ai_value = res.registers[i] * factor
                values.append(ai_value)

            response.success = True
            response.message = " | ".join(values)
            self.get_logger().info(f"Service Responce: {response.message}")
        else:
            response.success = False
            response.message = f"Failed to read AI register at address {addr}"
            self.get_logger().error(response.message)
        
        return response

    def ao_callback(self, msg, addr, chi):
        self.first_msg_received = True
        self.last_ao_time = self.get_clock().now()
        factor = self.get_parameter('ao_factor').value

        val_ua = int(msg.data * factor)
        res = self.client.write_registers(addr, [val_ua], device_id=1)

        if not res.isError():
            self.get_logger().info(f"AO{chi} OK: {val_ua} uA ditulis ke {addr}")
        else:
            self.get_logger().error(f"AO{chi} FAIL di register {addr}!")

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
            addr = self.get_parameter(f'ao{i+1}_address').value
            res = self.client.write_registers(addr, [0], device_id=1)
            if not res.isError():
                self.get_logger().info(f"AO{i+1} di-reset ke 0 uA untuk keamanan.")
            else:
                self.get_logger().error(f"Gagal mereset AO{i+1} di register {addr}!")

    def setMode_callback(self, request, response):
        # Alamat 0x0514 adalah 1300 desimal
        addr = 0x0514 
        
        # Pastikan nilainya Integer murni (bukan list!)
        mode_val = 1 if request.data else 0 
        
        try:
            # Perbaikan: Hapus kurung siku [] dan ganti device_id jadi slave
            res = self.client.write_register(addr, mode_val, device_id=1)

            if not res.isError():
                response.success = True
                response.message = f"Mode AO berhasil diubah ke {'4-20mA' if request.data else '0-20mA'}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f"Gagal nulis ke register {addr}. Cek koneksi!"
                self.get_logger().error(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Error Python: {str(e)}"
            self.get_logger().error(response.message)
            
        return response
    
    def reboot_callback(self, request, response):
        # Gunakan 2026 (offset dari 2027 di tabel)
        reboot_addr = 0x07EA
        # MAGIC VALUE: 0x5BB5 = 23477 desimal
        magic_reboot_value = 0x5BB5 
        
        self.get_logger().info(f"Mengirim kunci kontak {magic_reboot_value} ke register {reboot_addr}...")
        
        try:
            # WAJIB: write_register (FC 06) sesuai kolom 'Related function code'
            res = self.client.write_register(reboot_addr, magic_reboot_value, device_id=1)

            if not res.isError():
                response.success = True
                response.message = "Sinyal reboot diterima, ME31 akan restart dalam sekejap!"
            else:
                response.success = False
                response.message = "Alat nolak reboot. Cek apakah alamat 2026 sudah pas?"
        except Exception as e:
            response.success = False
            response.message = f"Error Kabel: {str(e)}"
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