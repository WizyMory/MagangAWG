#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout,
    QPushButton, QLabel, QLineEdit, QHBoxLayout,
    QComboBox, QTableWidget, QTableWidgetItem
)

from PyQt5.QtCore import QTimer

from project_interfaces.msg import DataConnectivity


class UiNode(Node):

    def __init__(self):
        super().__init__('ui_node')

        # publisher
        self.publisher = self.create_publisher(
            DataConnectivity,
            'modbus_command',
            10
        )

        # subscriber
        self.subscription = self.create_subscription(
            DataConnectivity,
            'modbus_response',
            self.response_callback,
            10
        )

        self.ui = None
        self.response_buffer = []

    def response_callback(self, msg):

        # simpan data hasil read
        self.response_buffer.append(
            (msg.reg_address, msg.value)
        )

        self.get_logger().info(
            f"Response: addr={msg.reg_address} value={msg.value}"
        )


class ModbusUI(QWidget):

    def __init__(self, node):
        super().__init__()

        self.node = node
        self.node.ui = self

        self.setWindowTitle("Modbus UI Controller")
        self.resize(750, 450)

        layout = QVBoxLayout()

        # =========================
        # CONNECTION SETTINGS
        # =========================

        layout.addWidget(QLabel("Serial Port"))
        self.port = QLineEdit("/dev/ttyUSB0")
        layout.addWidget(self.port)

        layout.addWidget(QLabel("Baudrate"))
        self.baud = QLineEdit("9600")
        layout.addWidget(self.baud)

        layout.addWidget(QLabel("Slave ID"))
        self.slave = QLineEdit()
        layout.addWidget(self.slave)

        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")

        conn_layout = QHBoxLayout()
        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.btn_disconnect)

        layout.addLayout(conn_layout)

        # =========================
        # FUNCTION SELECT
        # =========================

        func_layout = QHBoxLayout()

        self.fc_combo = QComboBox()
        self.fc_combo.addItems([
            "03 - Read Holding Registers",
            "04 - Read Input Registers",
            "06 - Write Single Register",
            "16 - Write Multiple Registers"
        ])

        self.address_input = QLineEdit("0")
        self.quantity_input = QLineEdit("1")
        self.value = QLineEdit("0")

        func_layout.addWidget(QLabel("Function"))
        func_layout.addWidget(self.fc_combo)

        func_layout.addWidget(QLabel("Address"))
        func_layout.addWidget(self.address_input)

        func_layout.addWidget(QLabel("Qty"))
        func_layout.addWidget(self.quantity_input)

        func_layout.addWidget(QLabel("Value"))
        func_layout.addWidget(self.value)

        layout.addLayout(func_layout)

        # =========================
        # BUTTONS
        # =========================

        self.btn_read = QPushButton("Read Register")
        self.btn_write = QPushButton("Write Register")
        self.btn_reboot = QPushButton("Reboot Slave")
        self.btn_restore = QPushButton("Factory Restore")
        self.btn_set_input = QPushButton("Set Input Range")
        self.btn_set_output = QPushButton("Set Output Range")

        layout.addWidget(self.btn_read)
        layout.addWidget(self.btn_write)
        layout.addWidget(self.btn_reboot)
        layout.addWidget(self.btn_restore)
        layout.addWidget(self.btn_set_input)
        layout.addWidget(self.btn_set_output)

        # =========================
        # TABLE
        # =========================

        self.table = QTableWidget()
        layout.addWidget(self.table)

        self.setLayout(layout)

        # =========================
        # CONNECT SIGNAL
        # =========================

        self.btn_connect.clicked.connect(self.publish_connect)
        self.btn_disconnect.clicked.connect(self.publish_disconnect)

        self.btn_read.clicked.connect(self.publish_read)
        self.btn_write.clicked.connect(self.publish_write)
        self.btn_reboot.clicked.connect(self.publish_reboot)
        self.btn_restore.clicked.connect(self.publish_restore)
        self.btn_set_input.clicked.connect(self.publish_set_input)
        self.btn_set_output.clicked.connect(self.publish_set_output)

        # =========================
        # TIMER UPDATE TABLE
        # =========================

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_table)
        self.timer.start(200)

    # =========================
    # UPDATE TABLE FROM BUFFER
    # =========================

    def refresh_table(self):

        if len(self.node.response_buffer) == 0:
            return

        data = self.node.response_buffer.copy()
        self.node.response_buffer.clear()

        self.update_table(data)

    # =========================
    # DISPLAY TABLE
    # =========================

    def update_table(self, data):

        self.table.setRowCount(len(data))
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Address", "Value"])

        for i, (addr, val) in enumerate(data):

            self.table.setItem(i, 0, QTableWidgetItem(str(addr)))
            self.table.setItem(i, 1, QTableWidgetItem(str(val)))

    # =========================
    # PUBLISH FUNCTIONS
    # =========================

    def publish_connect(self):

        msg = DataConnectivity()
        msg.port = self.port.text()
        msg.baudrate = int(self.baud.text())
        msg.mode = "connect"

        self.node.publisher.publish(msg)

    def publish_disconnect(self):

        msg = DataConnectivity()
        msg.mode = "disconnect"

        self.node.publisher.publish(msg)

    def publish_read(self):

        msg = DataConnectivity()

        msg.mode = "read"
        msg.port = self.port.text()
        msg.baudrate = int(self.baud.text())
        msg.slave_id = int(self.slave.text())
        msg.address = int(self.address_input.text())
        msg.quantity = int(self.quantity_input.text())

        fc_text = self.fc_combo.currentText()

        if "03" in fc_text:
            msg.function_code = 3

        elif "04" in fc_text:
            msg.function_code = 4

        self.node.publisher.publish(msg)

    def publish_write(self):

        msg = DataConnectivity()

        msg.mode = "write"
        msg.port = self.port.text()
        msg.baudrate = int(self.baud.text())
        msg.slave_id = int(self.slave.text())
        msg.address = int(self.address_input.text())
        msg.value = int(self.value.text())

        fc_text = self.fc_combo.currentText()

        if "06" in fc_text:
            msg.function_code = 6

        elif "16" in fc_text:
            msg.function_code = 16

        self.node.publisher.publish(msg)

    def publish_reboot(self):

        msg = DataConnectivity()
        msg.port = self.port.text()
        msg.baudrate = int(self.baud.text())
        msg.slave_id = int(self.slave.text())
        msg.mode = "reboot"

        self.node.publisher.publish(msg)

    def publish_restore(self):

        msg = DataConnectivity()
        msg.port = self.port.text()
        msg.baudrate = int(self.baud.text())
        msg.slave_id = int(self.slave.text())
        msg.mode = "restore"

        self.node.publisher.publish(msg)

    def publish_set_input(self):

        msg = DataConnectivity()
        msg.port = self.port.text()
        msg.baudrate = int(self.baud.text())
        msg.slave_id = int(self.slave.text())
        msg.value = float(self.value.text())
        msg.mode = "set_input_range"

        self.node.publisher.publish(msg)

    def publish_set_output(self):

        msg = DataConnectivity()
        msg.port = self.port.text()
        msg.baudrate = int(self.baud.text())
        msg.slave_id = int(self.slave.text())
        msg.value = float(self.value.text())
        msg.mode = "set_output_range"

        self.node.publisher.publish(msg)


# =========================
# MAIN
# =========================

def main(args=None):

    rclpy.init(args=args)

    node = UiNode()

    app = QApplication(sys.argv)
    window = ModbusUI(node)
    window.show()

    import threading
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
