import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from .modbus_func import ME31Handler
from std_msgs.msg import Int32, String, Empty

class BLDCMotorController(Node):
    def __init__(self):
        super().__init__('bldc_motor_controller')

        # PARAMETER DEFAULT
        self.declare_parameters(
            namespace= '',
            parameters = [
                ('port', '/dev/ttyUSB0'),
                ('slave_id', 1),
                ('baudrate', 19200),
                ('timeout', 1),
                ('parity', 'N'),
                ('stopbits', 1),
                ('bytesize', 8) 
                ])
        
        port = self.get_parameter('port').value
        self.slave_id = self.get_parameter('slave_id').value
        baud = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        parity = self.get_parameter('parity').value
        stopbits = self.get_parameter('stopbits').value
        bytesize = self.get_parameter('bytesize').value

        self.client = ModbusClient(port=port, baudrate=baud, timeout=timeout, parity=parity, stopbits=stopbits, bytesize=bytesize)
        self.handler = ME31Handler(self.client, self.get_logger())
        
        try:
            self.client.connect()
            self.get_logger().info("Modbus client connected successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to connect Modbus client: {e}")

        self.speed_sub = self.create_subscription(Int32, 'set_speed', self.speed_callback, 10)
        self.direction_sub = self.create_subscription(String, 'set_direction', self.direction_callback, 10)
        self.stop_sub = self.create_subscription(Empty, 'motor_stop', self.stop_callback, 10)
        self.enable_sub = self.create_subscription(Empty, 'enable_function_write', self.enable_callback, 10)
        
        self.timer = self.create_timer(1.0, self.read_actual_speed)
        self.state_timer = self.create_timer(5.0, self.motor_state_update)

    def speed_callback(self, msg):
        try: 
            self.get_logger().info(f"Received speed command: {msg.data}")
            self.set_speed(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error occurred while setting speed: {e}")

    def direction_callback(self, msg):
        try:
            self.get_logger().info(f"Received direction command: {msg.data}")
            self.set_direction(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error occurred while setting direction: {e}")

    def stop_callback(self, msg):
        try:
            self.get_logger().info("Received stop command")
            self.motor_stop()
        except Exception as e:
            self.get_logger().error(f"Error occurred while stopping motor: {e}")

    def enable_callback(self, msg):
        try:
            self.get_logger().info("Received enable function write command")
            self.enable_function_write()
        except Exception as e:
            self.get_logger().error(f"Error occurred while enabling function write: {e}")

    def set_speed(self, speed):
        # Assuming the speed register is at address 0x0001
        if self.client is None:
            self.get_logger().info("Modbus client is not initialized")

        success = self.handler.write_single(8193, speed)
        if success:
            self.get_logger().info(f"Speed set to {speed} successfully")
        else:
            self.get_logger().error(f"Failed to set speed to {speed}")

    def set_direction(self, direction):
        if self.client is None:
            self.get_logger().info("Modbus client is not initialized")

        if direction.lower() == 'forward':
            success = self.handler.write_single(8192, 1)
            self.get_logger().info("Direction set to forward")
        elif direction.lower() == 'reverse':
            success = self.handler.write_single(8192, 2)
            self.get_logger().info("Direction set to reverse")

        if success:
            self.get_logger().info(f"Direction set to {direction} successfully")
        else:
            self.get_logger().error(f"Failed to set direction to {direction}")

    def motor_stop(self):
        if self.client is None:
            self.get_logger().info("Modbus client is not initialized")

        success = self.handler.write_single(8192, 5)
        if success:
            self.get_logger().info("Motor stopped successfully")
        else:
            self.get_logger().error("Failed to stop the motor")

    def enable_function_write(self):
        if self.client is None:
            self.get_logger().info("Modbus client is not initialized")

        success = self.handler.write_single(8206, 0)
        if success:
            self.get_logger().info("Function write enabled successfully")
        else:
            self.get_logger().error("Failed to enable function write")

    def read_actual_speed(self):
        if self.client is None:
            self.get_logger().info("Modbus client is not initialized")

        speed = self.handler.read_multiple(12294, count=1, id=self.slave_id)

        shaft_speed = speed[0]/50

        if speed is not None:
            self.get_logger().info(f"Current speed: {shaft_speed}")
            return shaft_speed
        else:
            self.get_logger().error("Failed to read speed")
            return None
        
    def check_motor_state(self):
        if self.client is None:
            self.get_logger().info("Modbus client is not initialized")

        state = self.handler.read_multiple(8448, count=1, id=self.slave_id)

        if state[0] == 1:
            current_state = "Running Forward"
        elif state[0] == 2:
            current_state = "Running Reverse"
        elif state[0] == 3:
            current_state = "Shutting Down"
        elif state[0] == 4:
            current_state = "Drive Failure"
        elif state[0] == 5:
            current_state = "Drive OFF status"
        elif state[0] == 6:
            current_state = "Electrical Brake status"
        

        if state is not None:
            self.get_logger().info(f"Current motor state: {current_state}")
            return current_state
        else:
            self.get_logger().error("Failed to read motor state")
            return None
        
    def motor_state_update(self):
        self.check_motor_state()
        
def main(args=None):
    rclpy.init(args=args)
    node = BLDCMotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()