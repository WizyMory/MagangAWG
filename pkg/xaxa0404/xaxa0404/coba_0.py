#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QComboBox,
    QTableWidget, QTableWidgetItem
)

from easymodbus import modbusClient
from easymodbus.modbusClient import Parity, Stopbits


class ModbusPollUI(QWidget):

    def __init__(self):
        super().__init__()

        self.client = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("ME31 Modbus Tool")
        self.resize(700, 400)

        main_layout = QVBoxLayout()

        # ==============================
        # CONNECTION
        # ==============================
        conn_layout = QHBoxLayout()

        self.port_input = QLineEdit("/dev/ttyUSB0")
        self.baud_input = QLineEdit("9600")
        self.slave_input = QLineEdit("1")

        conn_layout.addWidget(QLabel("Port"))
        conn_layout.addWidget(self.port_input)

        conn_layout.addWidget(QLabel("Baudrate"))
        conn_layout.addWidget(self.baud_input)

        conn_layout.addWidget(QLabel("Slave ID"))
        conn_layout.addWidget(self.slave_input)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_modbus)
        conn_layout.addWidget(self.connect_btn)

        main_layout.addLayout(conn_layout)

        # ==============================
        # FUNCTION SECTION
        # ==============================
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

        func_layout.addWidget(QLabel("Function"))
        func_layout.addWidget(self.fc_combo)

        func_layout.addWidget(QLabel("Address"))
        func_layout.addWidget(self.address_input)

        func_layout.addWidget(QLabel("Qty/Value"))
        func_layout.addWidget(self.quantity_input)

        main_layout.addLayout(func_layout)

        # ==============================
        # BUTTONS
        # ==============================
        btn_layout = QHBoxLayout()

        self.read_btn = QPushButton("Read")
        self.read_btn.clicked.connect(self.read_data)

        self.write_btn = QPushButton("Write")
        self.write_btn.clicked.connect(self.write_data)

        btn_layout.addWidget(self.read_btn)
        btn_layout.addWidget(self.write_btn)

        main_layout.addLayout(btn_layout)

        # ==============================
        # TABLE OUTPUT
        # ==============================
        self.table = QTableWidget()
        main_layout.addWidget(self.table)

        self.setLayout(main_layout)

    # ==========================================
    # CONNECT
    # ==========================================
    def connect_modbus(self):
        try:
            port = self.port_input.text()
            baud = int(self.baud_input.text())

            self.client = modbusClient.ModbusClient(port)
            self.client.baudrate = baud
            self.client.parity = Parity.none
            self.client.stopbits = Stopbits.one

            self.client.connect()

            print("Connected to", port)

        except Exception as e:
            print("Connection failed:", e)

    # ==========================================
    # READ
    # ==========================================
    def read_data(self):
        if not self.client:
            print("Not connected")
            return

        try:
            slave = int(self.slave_input.text())
            address = int(self.address_input.text())
            qty = int(self.quantity_input.text())

            self.client.unitidentifier = slave

            fc = self.fc_combo.currentText()

            if "03" in fc:
                result = self.client.read_holding_registers(address, qty)

            elif "04" in fc:
                result = self.client.read_input_registers(address, qty)

            else:
                print("Invalid function for read")
                return

            self.display_table(result)

        except Exception as e:
            print("Read error:", e)

    # ==========================================
    # WRITE
    # ==========================================
    def write_data(self):
        if not self.client:
            print("Not connected")
            return

        try:
            slave = int(self.slave_input.text())
            address = int(self.address_input.text())
            value = int(self.quantity_input.text())

            self.client.unitidentifier = slave

            fc = self.fc_combo.currentText()

            if "06" in fc:
                self.client.write_single_register(address, value)

            elif "16" in fc:
                self.client.write_multiple_registers(address, [value])

            else:
                print("Invalid function for write")

            print("Write success")

        except Exception as e:
            print("Write error:", e)

    # ==========================================
    # DISPLAY TABLE
    # ==========================================
    def display_table(self, data):
        self.table.setRowCount(len(data))
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Index", "Value"])

        for i, val in enumerate(data):
            self.table.setItem(i, 0, QTableWidgetItem(str(i)))
            self.table.setItem(i, 1, QTableWidgetItem(str(val)))


# ==========================================
# MAIN FUNCTION
# ==========================================
def main():
    app = QApplication(sys.argv)
    window = ModbusPollUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
