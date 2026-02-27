#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import easymodbus.modbusClient
from easymodbus.modbusClient import convert_float_to_two_registers
from easymodbus.modbusClient import convert_registers_to_float

class subs_data(Node):

    def __init__(self):
        super().__init__("motor_pwm")
        self.modbus_client = self.connect_slave()
        # saran: pakai faster rate kalau mau RViz lebih halus, mis. 0.1
        self.create_timer(1/10, self.kirim_data, clock=self.get_clock())




    def connect_slave(self):
        try:
            client = easymodbus.modbusClient.ModbusClient('/dev/ttyUSB0')
            client.baudrate = 9600
            client.parity = easymodbus.modbusClient.Parity.none
            client.stopbits = easymodbus.modbusClient.Stopbits.one
            client.connect()
            # client.timeout = 50   # ms
            print("Connected to Modbus EBYTE")
            return client
        except Exception as e:
            print("Connection failed:", e)
            exit()


    def kirim_data(self):
        # write_float_multi_register(self.modbus_client, 2, 0, self.w_L)
        # write_float_multi_register(self.modbus_client, 2, 2, self.w_R)

        # baca dari slave (bisa None jika error)
        # self.X = read_float_holding_register(self.modbus_client, 2, 4, 2)
        # self.Y = read_float_holding_register(self.modbus_client, 2, 6, 2)
        # self.deg = read_float_holding_register(self.modbus_client, 2, 8, 2)

        # write_int_register(self.modbus_client, 1, 0x0064, 20000)
        # write_int_register(self.modbus_client, 1, 0x0065, 4000)
        # write_int_register(self.modbus_client, 1, 0x0066, 8760)
        # write_int_register(self.modbus_client, 1, 0x0067, 12340)

        write_int_register(self.modbus_client, 1, 0x0514, 1)
        write_int_register(self.modbus_client, 1, 0x0515, 1)
        write_int_register(self.modbus_client, 1, 0x0516, 1)
        write_int_register(self.modbus_client, 1, 0x0517, 1)

        # write_float_multi_register(self.modbus_client, 1, 0x0064, 12000)
        # write_float_multi_register(self.modbus_client, 1, 2, 9.5)
        # write_float_multi_register(self.modbus_client, 1, 4, 13.5)
        # write_float_multi_register(self.modbus_client, 1, 6, 17.4)

        # print(f"X={self.X}, Y={self.Y}, deg={self.deg}")
        # print(f"W_R={self.w_R}, W_L={self.w_L}")
        # print(f"v_R={self.v_R}, v_L={self.v_L}")

        # publish hanya jika nilai valid (tidak None)
        # if self.X is not None and self.Y is not None and self.deg is not None:
        # else:
        #     self.get_logger().warn("Odometry data None — skipping publish")


    

##### FUNGSI-FUNGSI KOMUNIKASI MODBUS #####
def write_float_multi_register(modbus_client, slave_id, start_address, value):
    try:
        modbus_client.unitidentifier = slave_id
        data = convert_float_to_two_registers(value)
        data = [data[1], data[0]]
        modbus_client.write_multiple_registers(start_address, data)
        print(f"Write OK → Slave:{slave_id} Addr:{start_address} Value:{value}")
    except Exception as e:
        print("Read Error:", e)
        return None


def write_int_register(modbus_client, slave_id, start_address, value):
    try:
        modbus_client.unitidentifier = slave_id
        modbus_client.write_single_register(start_address, value)
        print(f"Write OK → Slave:{slave_id} Addr:{hex(start_address)} Value:{value}")
    except Exception as e:
        print("Read Error:", e)
        return None
    
def read_float_holding_register(modbus_client, slave_id, start_address, quantity):
    try:
        modbus_client.unitidentifier = slave_id
        registers = modbus_client.read_holding_registers(start_address, quantity)
        value = convert_registers_to_float(registers)[0]
        return value
    except Exception as e:
        print("Read Error:", e)
        return None
    
def read_holding_register(modbus_client, slave_id, start_address, quantity):
    try:
        modbus_client.unitidentifier = slave_id
        registers = modbus_client.read_holding_registers(start_address, quantity)
        return registers
    except Exception as e:
        print("Read Error:", e)
        return None    
    


def main(args=None):
    rclpy.init(args=args)
    node = subs_data()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
