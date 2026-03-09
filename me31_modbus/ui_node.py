#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
import threading

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QComboBox, 
    QScrollArea, QSpinBox, QGroupBox, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer

from me31_msg.msg import InterfacesData


# =========================
# ROS2 NODE
# =========================

class UiNode(Node):
    def __init__(self):
        super().__init__('ui_node')

        # Publisher
        self.publisher = self.create_publisher(
            InterfacesData,
            'modbus_command',
            10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            InterfacesData,
            'modbus_response',
            self.response_callback,
            10
        )

        self.ui = None
        self.response_buffer = []

    def response_callback(self, msg):
        # Simpan data hasil read (termasuk slave_id agar bisa dipilah per container)
        self.response_buffer.append(
            (msg.slave_id, msg.address, msg.value)
        )

        self.get_logger().info(
            f"Response: slave={msg.slave_id} addr={msg.address} value={msg.value}"
        )


# =========================
# SLAVE CONTAINER WIDGET
# =========================

class SlaveContainer(QGroupBox):
    def __init__(self, slave_id, node, parent_ui):
        super().__init__(f"Slave ID: {slave_id}")
        
        self.slave_id = slave_id
        self.node = node
        self.parent_ui = parent_ui # Untuk referensi hapus diri sendiri dari main dict

        # Mapping address Modbus (Sesuaikan dengan address hardware aslinya)
        self.WRITE_MAP = {"AO1": 0, "AO2": 1, "AIO3": 2, "AO4": 3}
        self.READ_MAP = {"AI1": 0, "AI2": 1, "AI3": 2, "AI4": 3}

        # Reverse map untuk baca response
        self.REVERSE_READ_MAP = {v: k for k, v in self.READ_MAP.items()}

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # --------------------------------------------------------------
        # Settings: Port, Baudrate, Slave ID (Const)
        # --------------------------------------------------------------
        settings_layout = QHBoxLayout()
        
        self.port_input = QLineEdit("/dev/ttyUSB0")
        settings_layout.addWidget(QLabel("Port:"))
        settings_layout.addWidget(self.port_input)

        self.baudrate_input = QSpinBox()
        self.baudrate_input.setRange(9600, 115200)
        self.baudrate_input.setValue(9600)
        self.baudrate_input.setSingleStep(4800)
        settings_layout.addWidget(QLabel("Baudrate:"))
        settings_layout.addWidget(self.baudrate_input)

        self.id_label = QLabel(str(self.slave_id))
        self.id_label.setStyleSheet("font-weight: bold; color: blue;")
        settings_layout.addWidget(QLabel("Slave ID (Const):"))
        settings_layout.addWidget(self.id_label)

        main_layout.addLayout(settings_layout)

        # --------------------------------------------------------------
        # Output/Input Range Settings
        # --------------------------------------------------------------
        range_layout = QGridLayout()

        range_layout.addWidget(QLabel("Set Output Range:"), 0, 0)
        self.combo_out_range = QComboBox()
        self.combo_out_range.addItems(["0-20mA", "4-20mA"])
        range_layout.addWidget(self.combo_out_range, 0, 1)
        
        self.btn_set_out = QPushButton("Set")
        self.btn_set_out.clicked.connect(self.publish_set_output)
        range_layout.addWidget(self.btn_set_out, 0, 2)

        range_layout.addWidget(QLabel("Set Input Range:"), 1, 0)
        self.combo_in_range = QComboBox()
        self.combo_in_range.addItems(["0-20mA", "4-20mA"])
        range_layout.addWidget(self.combo_in_range, 1, 1)
        
        self.btn_set_in = QPushButton("Set")
        self.btn_set_in.clicked.connect(self.publish_set_input)
        range_layout.addWidget(self.btn_set_in, 1, 2)

        main_layout.addLayout(range_layout)

        # --------------------------------------------------------------
        # Connection Control Buttons
        # --------------------------------------------------------------
        btn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_reboot = QPushButton("Reboot")
        self.btn_restore = QPushButton("Restore")

        self.btn_connect.clicked.connect(self.publish_connect)
        self.btn_disconnect.clicked.connect(self.publish_disconnect)
        self.btn_reboot.clicked.connect(self.publish_reboot)
        self.btn_restore.clicked.connect(self.publish_restore)

        btn_layout.addWidget(self.btn_connect)
        btn_layout.addWidget(self.btn_disconnect)
        btn_layout.addWidget(self.btn_reboot)
        btn_layout.addWidget(self.btn_restore)
        main_layout.addLayout(btn_layout)

        # --------------------------------------------------------------
        # Write Output Section
        # --------------------------------------------------------------
        write_group = QGroupBox("Write Output")
        write_layout = QGridLayout()
        
        self.write_inputs = {}
        for i, target in enumerate(self.WRITE_MAP.keys()):
            lbl = QLabel(target)
            val_input = QLineEdit("0")
            btn_write = QPushButton("Write")
            
            btn_write.clicked.connect(lambda checked, t=target, inp=val_input: self.publish_write(t, inp.text()))

            write_layout.addWidget(lbl, i, 0)
            write_layout.addWidget(val_input, i, 1)
            write_layout.addWidget(btn_write, i, 2)
            
            self.write_inputs[target] = val_input

        write_group.setLayout(write_layout)
        main_layout.addWidget(write_group)

        # --------------------------------------------------------------
        # Read Input Section
        # --------------------------------------------------------------
        read_group = QGroupBox("Read Input")
        read_layout = QGridLayout()

        self.read_displays = {}
        for i, target in enumerate(self.READ_MAP.keys()):
            lbl = QLabel(target)
            val_display = QLineEdit()
            val_display.setReadOnly(True) 
            val_display.setPlaceholderText("Result...")
            btn_read = QPushButton("Read")

            btn_read.clicked.connect(lambda checked, t=target: self.publish_read(t))

            read_layout.addWidget(lbl, i, 0)
            read_layout.addWidget(val_display, i, 1)
            read_layout.addWidget(btn_read, i, 2)
            
            self.read_displays[target] = val_display

        read_group.setLayout(read_layout)
        main_layout.addWidget(read_group)

        # --------------------------------------------------------------
        # Close Container Button
        # --------------------------------------------------------------
        self.btn_close = QPushButton("Close Container")
        self.btn_close.setStyleSheet("background-color: #ff4d4d; color: white; font-weight: bold;")
        self.btn_close.clicked.connect(self.close_container)
        main_layout.addWidget(self.btn_close)

        self.setLayout(main_layout)

    # ------------------------------------------------------
    # Helper to create base message with common fields (port, baudrate, slave_id, mode)
    # ------------------------------------------------------
    def _create_base_msg(self, mode):
        msg = InterfacesData()
        msg.port = self.port_input.text()
        msg.baudrate = self.baudrate_input.value()
        msg.slave_id = self.slave_id
        msg.mode = mode
        return msg

    # -------------------------------------------------------
    # Publisher functions for different actions (connect, disconnect, reboot, restore, set range, write, read)
    # --------------------------------------------------------
    def publish_connect(self):
        msg = self._create_base_msg("connect")
        self.node.publisher.publish(msg)

    def publish_disconnect(self):
        msg = self._create_base_msg("disconnect")
        self.node.publisher.publish(msg)

    def publish_reboot(self):
        msg = self._create_base_msg("reboot")
        self.node.publisher.publish(msg)

    def publish_restore(self):
        msg = self._create_base_msg("restore")
        self.node.publisher.publish(msg)

    def publish_set_output(self):
        msg = self._create_base_msg("set_output_range")
        # Nilai index combobox: 0 (0-20mA), 1 (4-20mA)
        msg.value = self.combo_out_range.currentIndex() 
        self.node.publisher.publish(msg)

    def publish_set_input(self):
        msg = self._create_base_msg("set_input_range")
        msg.value = self.combo_in_range.currentIndex()
        self.node.publisher.publish(msg)

    def publish_write(self, target, value_str):
        msg = self._create_base_msg("write")
        msg.address = self.WRITE_MAP[target]
        msg.value = int(value_str) if value_str.isdigit() or (value_str.startswith('-') and value_str[1:].isdigit()) else 0
        msg.function_code = 6 # Asumsi Write Single Register
        self.node.publisher.publish(msg)

    def publish_read(self, target):
        msg = self._create_base_msg("read")
        msg.address = self.READ_MAP[target]
        msg.quantity = 1
        msg.function_code = 4 # Asumsi Read Input Registers
        self.node.publisher.publish(msg)

    # ----------------------------------------------
    # Update display value setelah dapat response dari node (dipanggil dari MainWindow saat refresh UI)
    # -----------------------------------------------
    def update_read_value(self, address, value):
        target = self.REVERSE_READ_MAP.get(address)
        if target and target in self.read_displays:
            self.read_displays[target].setText(str(value))

    def close_container(self):
        # Hapus diri sendiri dari dictionary di MainWindow
        if self.slave_id in self.parent_ui.containers:
            del self.parent_ui.containers[self.slave_id]
        self.deleteLater()


# =========================
# MAIN WINDOW CONTROLLER
# =========================
class ModbusUI(QWidget):
    def __init__(self, node):
        super().__init__()

        self.node = node
        self.node.ui = self

        self.setWindowTitle("Modbus UI Controller")
        self.resize(550, 800)

        # Dictionary untuk track slave yang aktif: {slave_id: SlaveContainer}
        self.containers = {} 

        layout = QVBoxLayout()

        # =========================
        # TOP BAR (Add Slave)
        # =========================
        top_layout = QHBoxLayout()
        self.slave_id_input = QSpinBox()
        self.slave_id_input.setRange(1, 255)
        self.slave_id_input.setPrefix("ID: ")
        
        self.btn_add_slave = QPushButton("Add Slave")
        self.btn_add_slave.clicked.connect(self.add_slave)

        top_layout.addWidget(self.slave_id_input)
        top_layout.addWidget(self.btn_add_slave)
        top_layout.addStretch()
        
        layout.addLayout(top_layout)

        # =========================
        # SCROLL AREA FOR CONTAINERS
        # =========================
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        
        self.container_widget = QWidget()
        self.container_layout = QVBoxLayout()
        self.container_layout.setAlignment(Qt.AlignTop)
        
        self.container_widget.setLayout(self.container_layout)
        self.scroll_area.setWidget(self.container_widget)
        
        layout.addWidget(self.scroll_area)
        self.setLayout(layout)

        # =========================
        # TIMER UPDATE BUFFER
        # =========================
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_ui)
        self.timer.start(200)

    def add_slave(self):
        slave_id = self.slave_id_input.value()
        
        # Mencegah ID kembar terbuka dua kali
        if slave_id in self.containers:
            self.node.get_logger().info(f"Slave ID {slave_id} is already opened!")
            return

        new_slave = SlaveContainer(slave_id, self.node, self)
        self.containers[slave_id] = new_slave
        self.container_layout.addWidget(new_slave)
        
        self.node.get_logger().info(f"Added Slave Container ID: {slave_id}")

    # =========================
    # UPDATE UI FROM BUFFER
    # =========================
    def refresh_ui(self):
        if len(self.node.response_buffer) == 0:
            return

        data = self.node.response_buffer.copy()
        self.node.response_buffer.clear()

        # Rute data respons ke container yang pas
        for slave_id, addr, val in data:
            if slave_id in self.containers:
                self.containers[slave_id].update_read_value(addr, val)


# =========================
# MAIN
# =========================
def main(args=None):
    rclpy.init(args=args)
    node = UiNode()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = ModbusUI(node)
    window.show()

    # Jalankan rclpy di thread terpisah agar tidak memblokir PyQt
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()