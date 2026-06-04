#!/usr/bin/env python3

import os
import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray

from .modbus_client import ModbusRelayClient
from .relay_map import RELAY_COIL_START, RELAY_COUNT
from .logger import RelayCSVLogger


class AMRRelayNode(Node):
    """
    AMR Relay Node – FINAL
    ---------------------
    - Topic based (no .srv)
    - Fault-latched Modbus
    - Safety OFF on exit
    """

    def __init__(self):
        super().__init__('amr_relay_node')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('slave_id', 1)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        slave = self.get_parameter('slave_id').value

        # ---------------- MODBUS ----------------
        self.modbus = ModbusRelayClient(port, baud, slave)
        self.modbus_ok = True
        self.modbus_fault = False

        # ---------------- LOGGER ----------------
        self.logger_csv = RelayCSVLogger(
            os.path.expanduser('~/amr_ws/src/amr_relay/logs')
        )

        # ---------------- STATE ----------------
        self.locked = False
        self.relay_state = [0] * RELAY_COUNT

        # ---------------- ROS I/O ----------------
        self.create_subscription(
            String, 'relay/cmd', self.cmd_callback, 10
        )
        self.pub_ack = self.create_publisher(
            String, 'relay/ack', 10
        )
        self.pub_status = self.create_publisher(
            Int32MultiArray, 'relay/status', 10
        )
        self.pub_conn   = self.create_publisher(
            String,'relay/connection', 10
        )

        # ---------------- TIMERS ----------------
        self.create_timer(1.0, self.publish_status)
        self.create_timer(1.0, self.publish_connection)
        self.create_timer(5.0, self.recover_modbus)

        # ---------------- SIGNAL SAFETY ----------------
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self.get_logger().info('AMR Relay Node ACTIVE')

    # =================================================
    # COMMAND HANDLER
    # =================================================
    def cmd_callback(self, msg: String):
        if self.locked:
            self.pub_ack.publish(String(data='BUSY'))
            return

        try:
            parts = msg.data.strip().split()
            if len(parts) != 2:
                raise ValueError('Format: "<ch> ON|OFF" or "ALL ON|OFF"')

            # -------- ALL ON / OFF --------
            if parts[0].upper() == 'ALL':
                if parts[1].upper() == 'ON':
                    self.modbus.all_on(RELAY_COIL_START, RELAY_COUNT)
                elif parts[1].upper() == 'OFF':
                    self.modbus.all_off(RELAY_COIL_START, RELAY_COUNT)
                else:
                    raise ValueError('Invalid ALL command')

                self.pub_ack.publish(String(data='OK'))
                self.logger_csv.log(-1, parts[1].upper() == 'ON', 'ALL')
                return

            # -------- SINGLE CHANNEL --------
            ch = int(parts[0])
            state = parts[1].upper() == 'ON'

            if ch < 0 or ch >= RELAY_COUNT:
                raise ValueError(f'Invalid channel {ch}')

            if not self.modbus_ok:
                raise RuntimeError('Modbus FAULT state')

            self.locked = True

            self.modbus.set_relay(RELAY_COIL_START + ch, state)
            self.relay_state[ch] = int(state)

            self.pub_ack.publish(String(data='OK'))
            self.logger_csv.log(ch, state, 'OK')

        except Exception as e:
            self.modbus_ok = False
            self.modbus_fault = True
            self.pub_ack.publish(String(data=f'ERROR {e}'))
            self.logger_csv.log(-1, False, f'ERROR {e}')

        finally:
            self.locked = False

    # =================================================
    # STATUS POLLING
    # =================================================
    def publish_status(self):
        if not self.modbus_ok or self.modbus_fault:
            return

        try:
            states = self.modbus.read_relays(
                RELAY_COIL_START,
                RELAY_COUNT
            )

            self.relay_state = [int(s) for s in states]
            msg = Int32MultiArray()
            msg.data = self.relay_state
            self.pub_status.publish(msg)

        except Exception as e:
            self.modbus_ok = False
            self.modbus_fault = True
            self.get_logger().error(f'Status read error: {e}')

    def publish_connection(self):
        if self.modbus_fault:
            status = 'FAULT'
        elif self.modbus_ok:
            status = 'OK'
        else:
            status = 'DISCONNECTED'
        self.pub_conn.publish(String(data=status))

    # =================================================
    # MODBUS RECOVERY
    # =================================================
    def recover_modbus(self):
        if not self.modbus_fault:
            return

        self.get_logger().warn('Attempting Modbus recovery...')
        try:
            self.modbus.close()
            time.sleep(0.5)
            self.modbus = ModbusRelayClient(
                self.get_parameter('port').value,
                self.get_parameter('baudrate').value,
                self.get_parameter('slave_id').value
            )
            self.modbus_ok = True
            self.modbus_fault = False
            self.get_logger().info('Modbus recovered')

        except Exception as e:
            self.get_logger().error(f'Modbus recovery failed: {e}')

    # =================================================
    # SAFETY OFF
    # =================================================
    def safety_off(self):
        self.get_logger().warn('SAFETY OFF → ALL RELAYS OFF')
        try:
            self.modbus.all_off(RELAY_COIL_START, RELAY_COUNT)
        except Exception:
            pass

    # =================================================
    # SIGNAL HANDLER
    # =================================================
    def _signal_handler(self, signum, frame):
        self.get_logger().warn(f'Signal {signum} received')
        self.safety_off()
        self.logger_csv.close()
        raise KeyboardInterrupt

    def destroy_node(self):
        self.get_logger().warn('Node destroy → ALL RELAYS OFF')
        self.safety_off()
        self.logger_csv.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = AMRRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()