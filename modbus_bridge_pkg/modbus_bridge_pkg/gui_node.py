import sys
import tkinter as tk
from tkinter import messagebox
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float32
import threading

class ModbusGui(Node):
    def __init__(self):
        super().__init__('modbus_gui_node')
        
        # Setup Clients
        self.reboot_cli = self.create_client(Trigger, 'reboot_device')
        self.mode_cli = self.create_client(SetBool, 'set_ao_mode')

        # Inisialisasi Window Tkinter
        self.root = tk.Tk()
        self.root.title("ME31 Control Dashboard")
        self.root.geometry("300x400")

        self._build_ui()

    def _build_ui(self):
        # Label Judul
        tk.Label(self.root, text="ME31 MODBUS CONTROL", font=('Arial', 12, 'bold')).pack(pady=10)

        # Tombol Reboot
        tk.Button(self.root, text="REBOOT DEVICE", command=self.send_reboot, 
                  bg="red", fg="white", height=2, width=20).pack(pady=10)

        # Tombol Ganti Mode
        tk.Label(self.root, text="Hardware Mode:").pack()
        tk.Button(self.root, text="Set 4-20mA", command=lambda: self.send_mode(True), 
                  bg="blue", fg="white", width=15).pack(pady=2)
        tk.Button(self.root, text="Set 0-20mA", command=lambda: self.send_mode(False), 
                  bg="gray", fg="white", width=15).pack(pady=2)

    def send_reboot(self):
        if not self.reboot_cli.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Service Reboot Tidak Aktif!")
            return
        self.reboot_cli.call_async(Trigger.Request())
        messagebox.showinfo("Sent", "Sinyal Reboot Terkirim!")

    def send_mode(self, mode_bool):
        req = SetBool.Request(data=mode_bool)
        self.mode_cli.call_async(req)
        status = "4-20mA" if mode_bool else "0-20mA"
        messagebox.showinfo("Sent", f"Mode has been changed to {status}, reboot device for changes to take effect.")
        self.get_logger().info(f"Mengubah mode ke {status}")

    def send_ao(self):
        val = float(self.ao_slider.get())
        self.ao_pub.publish(Float32(data=val))

    def run_gui(self):
        # Jalankan loop Tkinter
        self.root.mainloop()

def main():
    rclpy.init()
    node = ModbusGui()
    
    # Jalankan ROS 2 spin di thread terpisah agar GUI tidak hang
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    node.run_gui()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()