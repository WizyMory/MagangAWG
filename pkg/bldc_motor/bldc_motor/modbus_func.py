import rclpy
from rclpy.node import Node

class ME31Handler:
    def __init__(self, client, logger):
        self.client = client
        self.log = logger
        self.device_id = 1

    def write_multiple(self, address, value):
        try:
            res = self.client.write_registers(address, [value], device_id=self.device_id)
            if res.isError():
                self.log.error(f"Failed to write value {value} to register {address}: {res}")
                return False
            return True
        except Exception as e:
            self.log.error(f"Exception while writing to register {address}: {e}")
            return False
        
    def write_single(self, address, value):
        try:
            res = self.client.write_register(address, value, device_id=self.device_id)
            if res.isError():
                self.log.error(f"Failed to write value {value} to register {address}: {res}")
                return False
            return True
        except Exception as e:
            self.log.error(f"Exception while writing to register {address}: {e}")
            return False
        
    def read_single(self, address, count, id):
        try:
            res = self.client.read_input_registers(address, count=count, device_id=id)
            if res.isError():
                self.log.error(f"Failed to read registers starting at {address}: {res}")
                return None
            return res.registers
        except Exception as e:
            self.log.error(f"Exception while reading registers starting at {address}: {e}")
            return None

    def read_multiple(self, address, count, id):
        try:
            res = self.client.read_holding_registers(address, count=count, device_id=id)
            if res.isError():
                self.log.error(f"Failed to read registers starting at {address}: {res}")
                return None
            return res.registers
        except Exception as e:
            self.log.error(f"Exception while reading registers starting at {address}: {e}")
            return None
    