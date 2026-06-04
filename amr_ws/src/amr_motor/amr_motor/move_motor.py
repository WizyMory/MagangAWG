import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger

from tf2_ros import TransformBroadcaster
from diagnostic_updater import Updater, DiagnosticStatusWrapper

import math

from amr_motor.modbus_rtu_client import ModbusRTUClient
from amr_motor.zlac8015d_v0 import ZLAC8015D
from amr_motor.diff_drive import twist_to_rpm


class AMRMotorNode(Node):
    """
    AMR MOTOR NODE – DRIVER ACCEL + ENCODER ODOMETRY
    """

    def __init__(self):
        super().__init__('amr_motor')

        # =====================================================
        # SHUTDOWN FLAG
        # =====================================================
        self._shutting_down = False

        # =====================================================
        # PARAMETERS
        # =====================================================
        self.declare_parameters('', [
            ('port', '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0002T4J-if00-port0'),
            ('baudrate', 115200),
            ('slave_id', 1),

            ('wheel_radius', 0.065),
            ('wheel_base', 0.3118),

            ('encoder_cpr', 4096),
            ('gear_ratio', 1.0),

            ('cmd_vel_timeout', 0.2),
            ('max_rpm', 3000),

            ('accel_ms', 1000),
            ('decel_ms', 800),
            ('quick_stop_ms', 10),
        ])

        # =====================================================
        # MODBUS + DRIVER
        # =====================================================
        self.mb = ModbusRTUClient(
            port=self.get_parameter('port').value,
            baudrate=self.get_parameter('baudrate').value
        )
        self.driver = ZLAC8015D(self.mb, self.get_parameter('slave_id').value)

        self.driver.set_velocity_mode()
        self.driver.set_accel_profile(
            accel_ms=self.get_parameter('accel_ms').value,
            decel_ms=self.get_parameter('decel_ms').value,
            quick_stop_ms=self.get_parameter('quick_stop_ms').value
        )
        self.driver.stop()

        self.get_logger().info(
            f"ZLAC accel profile: accel={self.get_parameter('accel_ms').value} ms, "
            f"decel={self.get_parameter('decel_ms').value} ms"
        )

        # =====================================================
        # STATE
        # =====================================================
        self.motor_enabled = False
        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.cmd_vel_received = False

        self.target_rpm_l = 0
        self.target_rpm_r = 0
        self.sent_rpm_l = 0
        self.sent_rpm_r = 0

        self.enc_l = None
        self.enc_r = None
        self.prev_enc_l = None
        self.prev_enc_r = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = self.get_clock().now()

        self.bus_voltage = None
        self.error_code_l = 0
        self.error_code_r = 0

        # =====================================================
        # ROS INTERFACE
        # =====================================================
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_service(SetBool, '/amr_motor/enable', self.enable_srv_cb)
        self.create_service(Trigger, '/amr_motor/reset_odometry', self.reset_odometry_cb)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # =====================================================
        # TIMERS
        # =====================================================
        # Format: create_timer(period_in_seconds, callback)
        # Rumus:
        #   frequency (Hz) = 1 / period (detik)
        #   period (detik) = 1 / frequency (Hz)
        #
        # Catatan umum:
        # - Loop komputasi boleh cepat
        # - Loop Modbus (serial) jangan terlalu cepat
        # - Monitoring bisa lambat

        self.create_timer(0.02, self.control_loop) # 0.02 s → 50 Hz
        self.create_timer(0.05, self.modbus_write_loop)
        self.create_timer(0.05, self.read_motion_state_timer)
        self.create_timer(0.05, self.update_odometry)
        self.create_timer(2.0, self.read_voltage_timer)
        self.create_timer(1.0, self.read_error_code_timer)

        # =====================================================
        # DIAGNOSTICS
        # =====================================================
        self.updater = Updater(self)
        self.updater.setHardwareID("ZLAC8015D")
        self.updater.add("Motor Driver", self.diagnostic_cb)
        self.create_timer(1.0, self.updater.update)

        self.get_logger().info("AMR Motor ready (DISABLED)")

    # =====================================================
    # CALLBACKS
    # =====================================================
    def enable_srv_cb(self, req, resp):
        if req.data:
            self.driver.clear_fault()
            self.driver.enable()
            self.motor_enabled = True
            resp.message = "Motor ENABLED"
            self.get_logger().info(resp.message)
        else:
            self.driver.set_speed(0, 0)
            self.driver.stop()
            self.motor_enabled = False
            self.sent_rpm_l = 0
            self.sent_rpm_r = 0
            resp.message = "Motor DISABLED"
            self.get_logger().warn(resp.message)

        resp.success = True
        return resp

    def cmd_cb(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.cmd_vel_received = True

    # =====================================================
    def control_loop(self):
        if self._shutting_down or not self.motor_enabled:
            self.target_rpm_l = 0
            self.target_rpm_r = 0
            return

        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        rpm_l, rpm_r = 0, 0

        if self.cmd_vel_received and dt <= self.get_parameter('cmd_vel_timeout').value:
            rpm_l, rpm_r = twist_to_rpm(
                self.cmd.linear.x,
                self.cmd.angular.z,
                self.get_parameter('wheel_radius').value,
                self.get_parameter('wheel_base').value
            )

        max_rpm = self.get_parameter('max_rpm').value
        self.target_rpm_l = -int(max(min(rpm_l, max_rpm), -max_rpm))
        self.target_rpm_r = int(max(min(rpm_r, max_rpm), -max_rpm))

    # =====================================================
    def modbus_write_loop(self):
        if self._shutting_down or not self.motor_enabled:
            return
        try:
            if (self.target_rpm_l != self.sent_rpm_l or
                self.target_rpm_r != self.sent_rpm_r):
                self.driver.set_speed(self.target_rpm_l, self.target_rpm_r)
                self.sent_rpm_l = self.target_rpm_l
                self.sent_rpm_r = self.target_rpm_r
        except KeyboardInterrupt:
            return
        except Exception:
            pass

    # =====================================================
    def read_motion_state_timer(self):
        if self._shutting_down:
            return
        try:
            _, _ = self.driver.read_actual_speed()
            self.enc_l, self.enc_r = self.driver.read_encoder()
        except KeyboardInterrupt:
            return
        except Exception as e:
            self.get_logger().warn(f"Failed to read motion state: {e}")

    # =====================================================
    """
    def update_odometry(self):
        if self._shutting_down or self.enc_l is None or self.enc_r is None:
            return

        if self.prev_enc_l is None:
            self.prev_enc_l = self.enc_l
            self.prev_enc_r = self.enc_r
            self.last_odom_time = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = now
        if dt <= 0:
            return

        dl = -(self.enc_l - self.prev_enc_l)
        dr = (self.enc_r - self.prev_enc_r)
        self.prev_enc_l = self.enc_l
        self.prev_enc_r = self.enc_r

        r = self.get_parameter('wheel_radius').value
        b = self.get_parameter('wheel_base').value
        cpr = self.get_parameter('encoder_cpr').value

        ds_l = r * (2 * math.pi * dl / cpr)
        ds_r = r * (2 * math.pi * dr / cpr)

        ds = (ds_l + ds_r) / 2.0
        dtheta = (ds_r - ds_l) / b

        self.x += ds * math.cos(self.yaw + dtheta / 2.0)
        self.y += ds * math.sin(self.yaw + dtheta / 2.0)
        self.yaw += dtheta

        self.publish_odom(ds / dt, dtheta / dt)

    # =====================================================

    def publish_odom(self, v, w):
        # AMBIL WAKTU SEKALI
        #stamp = self.get_clock().now().to_msg()
        
        now = self.get_clock().now()
        self.last_odom_time = now

        stamp = now.to_msg()


        q = Quaternion()
        q.z = math.sin(self.yaw / 2.0)
        q.w = math.cos(self.yaw / 2.0)

        # -------------------------
        # ODOMETRY MESSAGE
        # -------------------------
        odom = Odometry()
        odom.header.stamp = stamp          # ✅ sama
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # -------------------------
        # TF MESSAGE
        # -------------------------
        t = TransformStamped()
        t.header.stamp = stamp              # ✅ sama persis
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)
    """    
    def update_odometry(self):
        if self._shutting_down:
            return

        now = self.get_clock().now()

        # =====================================================
        # FALLBACK: tetap publish TF odom -> base_link
        # walaupun encoder belum siap
        # =====================================================
        if self.enc_l is None or self.enc_r is None:
            self.publish_odom(now, 0.0, 0.0)
            return

        if self.prev_enc_l is None:
            self.prev_enc_l = self.enc_l
            self.prev_enc_r = self.enc_r
            self.last_odom_time = now

            # publish TF pertama kali supaya frame odom langsung muncul
            self.publish_odom(now, 0.0, 0.0)
            return

        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # encoder delta
        dl = -(self.enc_l - self.prev_enc_l)
        dr =  (self.enc_r - self.prev_enc_r)

        self.prev_enc_l = self.enc_l
        self.prev_enc_r = self.enc_r
        self.last_odom_time = now

        r = self.get_parameter('wheel_radius').value
        b = self.get_parameter('wheel_base').value
        cpr = self.get_parameter('encoder_cpr').value

        ds_l = r * (2.0 * math.pi * dl / cpr)
        ds_r = r * (2.0 * math.pi * dr / cpr)

        ds = 0.5 * (ds_l + ds_r)
        dtheta = (ds_r - ds_l) / b

        self.x += ds * math.cos(self.yaw + 0.5 * dtheta)
        self.y += ds * math.sin(self.yaw + 0.5 * dtheta)
        self.yaw += dtheta

        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        v = ds / dt
        w = dtheta / dt

        self.publish_odom(now, v, w)

    def publish_odom(self, stamp_time, v, w):
        stamp = stamp_time.to_msg()

        q = Quaternion()
        q.z = math.sin(self.yaw * 0.5)
        q.w = math.cos(self.yaw * 0.5)

        # -------------------------
        # Odometry
        # -------------------------
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # -------------------------
        # TF
        # -------------------------
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)


    # =====================================================
    def reset_odometry_cb(self, req, resp):
        if self.enc_l is None or self.enc_r is None:
            resp.success = False
            resp.message = "Encoder not ready"
            return resp

        self.x = self.y = self.yaw = 0.0
        self.prev_enc_l = self.enc_l
        self.prev_enc_r = self.enc_r
        self.last_odom_time = self.get_clock().now()

        resp.success = True
        resp.message = "Odometry reset"
        return resp

    # =====================================================
    def read_voltage_timer(self):
        if self._shutting_down:
            return
        try:
            self.bus_voltage = self.driver.read_voltage()
        except KeyboardInterrupt:
            return
        except Exception:
            pass

    def read_error_code_timer(self):
        if self._shutting_down:
            return
        try:
            self.error_code_l, self.error_code_r = self.driver.read_error_code()
        except KeyboardInterrupt:
            return
        except Exception:
            pass

    def diagnostic_cb(self, stat: DiagnosticStatusWrapper):
        stat.summary(
            stat.OK if self.motor_enabled else stat.WARN,
            "Motor ENABLED" if self.motor_enabled else "Motor DISABLED"
        )
        return stat



# =====================================================
def main():
    rclpy.init()
    node = AMRMotorNode()

    def shutdown_hook():
        node.get_logger().warn("ROS shutdown → stop motor")
        node._shutting_down = True
        try:
            node.driver.set_speed(0, 0)
            node.driver.stop()
        except Exception:
            pass

    rclpy.get_default_context().on_shutdown(shutdown_hook)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()