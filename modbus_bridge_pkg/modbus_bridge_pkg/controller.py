import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, Trigger

class AOController(Node):
    def __init__(self):
        super().__init__('ao_controller')

        self.ao_publishers = {}
        for i in range(4):
            topic_name = f"/AO{i+1}"
            self.ao_publishers[i+1] = self.create_publisher(Float32, 
                                  topic_name, 
                                  10)
        
        self.create_timer(8.0, self.test_logic)

        self.mode_client = self.create_client(SetBool, 'set_ao_mode')
        self.reboot_client = self.create_client(Trigger, 'reboot_device')

        self.get_logger().info("Controller is running...")

    def write_ao(self, channel, value):
        if channel in self.ao_publishers:
            msg = Float32(data = float(value))
            self.ao_publishers[channel].publish(msg)
            self.get_logger().info(f"Published {value} to {channel}")
        else:
            self.get_logger().error(f"Channel {channel} not found")

    def test_logic(self):
        self.write_ao(1, 0.0)
        self.write_ao(2, 0.0)
        self.write_ao(3, 0.0)
        self.write_ao(4, 0.0)

    def change_hardware_mode(self, mode):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_ao_mode service...')
        
        req = SetBool.Request(data=mode)
        res = self.mode_client.call_async(req)
        res.add_done_callback(self.mode_response_callback)

    def reboot_dev(self):
        if not self.reboot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Reboot service not available!')
            return
        
        req = Trigger.Request()
        res = self.reboot_client.call_async(req)
        res.add_done_callback(self.reboot_response_callback)
    
    def _common_response_callback(self, res):
        try:
            response = res.result()
            if response.success:
                self.get_logger().info(f"Service call successful: {response.message}")
            else:
                self.get_logger().error(f"Service call failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AOController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()