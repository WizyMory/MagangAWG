#!/usr/bin/env python3
"""
AMR Magnetic Node – CCF-NS16 Magnetic Line Sensor
PT. Adhikara Wiyasa Gani

Topics publish ke web UI:
  magnetic/status      Float32MultiArray  → data sensor real-time
  magnetic/connection  String             → "OK" | "FAULT" | "DISCONNECTED"
  magnetic/ack         String             → "OK" | "ERROR ..."
  magnetic/log         String (JSON)      → log entry untuk UI

Topic subscribe dari web UI:
  magnetic/cmd         String (JSON)      → command dari magnetic.html

Format magnetic/status (21 float):
  [0]   line_detected    (0=false, 1=true)
  [1]   median_value     (float, e.g. 8.5)
  [2]   median_integer   (int)
  [3]   median_decimal   (int)
  [4]   active_count     (int)
  [5]   lateral_error_mm (float, 9999.0 jika N/A)
  [6]   quality_code     (0=NO_LINE, 1=GOOD, 2=WIDE_LINE, 3=TOO_MANY_POINTS)
  [7]   position_mask    (int, raw 16-bit)
  [8]   address          (int, slave ID)
  [9]   raw_reg0         (int)
  [10]  raw_reg1         (int)
  [11..26]  active_points[0..15]  (1 jika aktif, 0 jika tidak, P1~P16)

Format magnetic/cmd (JSON):
  { "action": "set_address",   "address": 1 }
  { "action": "set_baudrate",  "baudrate": 9600 }
  { "action": "set_mode",      "mode": "response" | "continuous" | "change" }
  { "action": "set_frequency", "frequency": 10 | 25 | 50 | 100 }
  { "action": "read_address" }
"""

import json
import os
import csv
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

from pymodbus.client import ModbusSerialClient


# ═══════════════════════════════════════════════════════════
# CONSTANTS
# ═══════════════════════════════════════════════════════════

SENSOR_COUNT    = 16
SENSOR_SPACING  = 10.0   # mm
CENTER_POINT    = 8.5

class Reg:
    SENSOR_DATA      = 0x0000
    ADDRESS          = 0x0002
    BAUDRATE         = 0x0003
    OUTPUT_MODE      = 0x0004
    OUTPUT_FREQUENCY = 0x0005

class OutputMode:
    CHANGE     = 0x0001
    CONTINUOUS = 0x0002
    RESPONSE   = 0x0003

BAUDRATE_CODE = {
    2400:   0x0001,
    4800:   0x0002,
    9600:   0x0003,
    19200:  0x0004,
    38400:  0x0005,
    57600:  0x0006,
    115200: 0x0007,
}

FREQUENCY_CODE = {
    10:  0x0001,
    25:  0x0002,
    50:  0x0003,
    100: 0x0004,
}

QUALITY_CODE = {
    'NO_LINE':         0,
    'GOOD':            1,
    'WIDE_LINE':       2,
    'TOO_MANY_POINTS': 3,
}

MODE_MAP = {
    'response':   OutputMode.RESPONSE,
    'continuous': OutputMode.CONTINUOUS,
    'change':     OutputMode.CHANGE,
}


# ═══════════════════════════════════════════════════════════
# SENSOR DRIVER
# ═══════════════════════════════════════════════════════════

class MagneticSensorDriver:
    """
    Low-level driver untuk CCF-NS16 Magnetic Navigation Sensor.
    Didesain untuk dipakai dalam ROS2 node (non-pymodbus context manager).
    Pakai pymodbus ModbusSerialClient langsung.
    """

    def __init__(
        self,
        port:         str   = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0',
        baudrate:     int   = 9600,
        slave_id:     int   = 1,
        timeout_s:    float = 1.0,
        retries:      int   = 3,
        retry_delay:  float = 0.03,
        bit_logic:    str   = 'active_low',   # 'active_low' | 'active_high'
        invert_error: bool  = False,
        fallback_median: bool = True,
    ):
        self.port           = port
        self.baudrate       = baudrate
        self.slave_id       = slave_id
        self.timeout_s      = timeout_s
        self.retries        = retries
        self.retry_delay    = retry_delay
        self.bit_logic      = bit_logic
        self.invert_error   = invert_error
        self.fallback_median = fallback_median

        self._client: ModbusSerialClient = None
        self._lock = threading.Lock()

    # ── Connection ─────────────────────────────────────────

    def connect(self) -> bool:
        try:
            self._client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=self.timeout_s,
            )
            return self._client.connect()
        except Exception:
            return False

    def close(self):
        try:
            if self._client:
                self._client.close()
        except Exception:
            pass

    def reconnect(self) -> bool:
        self.close()
        time.sleep(0.2)
        return self.connect()

    # ── Low-level Modbus ───────────────────────────────────

    def _read_regs(self, address: int, count: int):
        """FC03 read dengan retry."""
        with self._lock:
            for attempt in range(self.retries):
                try:
                    r = self._client.read_holding_registers(
                        address=address, count=count, slave=self.slave_id)
                    if r is not None and not r.isError():
                        return list(r.registers)
                except Exception:
                    pass
                time.sleep(self.retry_delay)
        return None

    def _write_reg(self, address: int, value: int) -> bool:
        """FC06 write dengan retry."""
        with self._lock:
            for attempt in range(self.retries):
                try:
                    r = self._client.write_register(
                        address=address, value=value & 0xFFFF, slave=self.slave_id)
                    if r is not None and not r.isError():
                        return True
                except Exception:
                    pass
                time.sleep(self.retry_delay)
        return False

    # ── High-level API ─────────────────────────────────────

    def read_sensor(self) -> dict:
        """
        Baca 2 register sensor, decode, return dict.
        reg0: high byte = median integer, low byte = median decimal
        reg1: 16-bit position mask
        """
        regs = self._read_regs(Reg.SENSOR_DATA, 2)
        if not regs or len(regs) < 2:
            return None

        reg0, reg1 = regs[0], regs[1]

        median_integer = (reg0 >> 8) & 0xFF
        median_decimal = reg0 & 0xFF
        median_value   = float(median_integer) + (float(median_decimal) / 10.0)
        position_mask  = reg1 & 0xFFFF

        # Decode active points
        active_points = []
        for bit in range(SENSOR_COUNT):
            bit_is_one = bool(position_mask & (1 << bit))
            is_active  = (not bit_is_one) if self.bit_logic == 'active_low' else bit_is_one
            if is_active:
                active_points.append(bit + 1)  # 1-based

        active_count  = len(active_points)
        line_detected = active_count > 0

        # Fallback median
        if self.fallback_median and line_detected and median_value <= 0.0 and active_points:
            median_value   = 0.5 * (active_points[0] + active_points[-1])
            median_integer = int(median_value)
            median_decimal = int(round((median_value - median_integer) * 10))

        # Lateral error
        if line_detected:
            lateral_error_mm = (median_value - CENTER_POINT) * SENSOR_SPACING
            if self.invert_error:
                lateral_error_mm = -lateral_error_mm
        else:
            lateral_error_mm = None

        # Quality
        if active_count == 0:
            quality = 'NO_LINE'
        elif active_count <= 4:
            quality = 'GOOD'
        elif active_count <= 8:
            quality = 'WIDE_LINE'
        else:
            quality = 'TOO_MANY_POINTS'

        return {
            'line_detected':    line_detected,
            'median_value':     median_value,
            'median_integer':   median_integer,
            'median_decimal':   median_decimal,
            'active_points':    active_points,
            'active_count':     active_count,
            'lateral_error_mm': lateral_error_mm,
            'quality':          quality,
            'position_mask':    position_mask,
            'raw_reg0':         reg0,
            'raw_reg1':         reg1,
        }

    def set_address(self, new_address: int) -> bool:
        if not 1 <= new_address <= 254:
            raise ValueError(f"Address harus 1..254, dapat {new_address}")
        return self._write_reg(Reg.ADDRESS, new_address)

    def set_baudrate(self, baudrate: int) -> bool:
        if baudrate not in BAUDRATE_CODE:
            raise ValueError(f"Baudrate tidak didukung: {baudrate}")
        return self._write_reg(Reg.BAUDRATE, BAUDRATE_CODE[baudrate])

    def set_output_mode(self, mode_str: str) -> bool:
        if mode_str not in MODE_MAP:
            raise ValueError(f"Mode tidak valid: {mode_str}")
        return self._write_reg(Reg.OUTPUT_MODE, MODE_MAP[mode_str])

    def set_output_frequency(self, freq_hz: int) -> bool:
        if freq_hz not in FREQUENCY_CODE:
            raise ValueError(f"Frequency tidak didukung: {freq_hz}")
        return self._write_reg(Reg.OUTPUT_FREQUENCY, FREQUENCY_CODE[freq_hz])

    def read_address_register(self) -> int:
        """Baca register address sensor (0x0002)."""
        r = self._read_regs(Reg.ADDRESS, 1)
        return r[0] if r else None


# ═══════════════════════════════════════════════════════════
# CSV LOGGER
# ═══════════════════════════════════════════════════════════

class MagneticCSVLogger:
    """Log aktivitas magnetic sensor ke CSV."""

    def __init__(self, log_dir: str):
        os.makedirs(log_dir, exist_ok=True)
        fname = datetime.now().strftime('%Y-%m-%d') + '_magnetic.csv'
        self._path = os.path.join(log_dir, fname)
        self._lock = threading.Lock()
        if not os.path.exists(self._path):
            with open(self._path, 'w', newline='') as f:
                csv.writer(f).writerow(['time', 'tag', 'level', 'msg'])

    def log(self, tag: str, level: str, msg: str):
        ts = datetime.now().isoformat(timespec='microseconds')
        with self._lock:
            try:
                with open(self._path, 'a', newline='') as f:
                    csv.writer(f).writerow([ts, tag, level, msg])
            except Exception:
                pass

    def close(self):
        pass


# ═══════════════════════════════════════════════════════════
# ROS2 NODE
# ═══════════════════════════════════════════════════════════

class AMRMagneticNode(Node):
    """
    ROS2 Node untuk CCF-NS16 Magnetic Line Sensor.
    Konsep sama dengan AMRMotorNode / AMRRelayNode.

    Topics publish:
      magnetic/status     Float32MultiArray  — data sensor (21 float)
      magnetic/connection String             — OK | FAULT | DISCONNECTED
      magnetic/ack        String             — response setiap command
      magnetic/log        String (JSON)      — log entry untuk web UI

    Topic subscribe:
      magnetic/cmd        String (JSON)      — command dari web UI
    """

    def __init__(self):
        super().__init__('amr_magnetic')

        self._shutting_down = False

        # ─── Parameters ────────────────────────────────────
        self.declare_parameters('', [
            ('port',          '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'),
            ('baudrate',      9600),
            ('slave_id',      1),
            ('timeout_s',     1.0),
            ('retries',       3),
            ('bit_logic',     'active_low'),
            ('invert_error',  False),
            ('fallback_median', True),
            ('poll_rate_hz',  10.0),
            ('log_dir',       os.path.expanduser('~/amr_ws/src/amr_magnetic/logs')),
        ])

        # ─── Driver ────────────────────────────────────────
        self.sensor = MagneticSensorDriver(
            port          = self.get_parameter('port').value,
            baudrate      = self.get_parameter('baudrate').value,
            slave_id      = self.get_parameter('slave_id').value,
            timeout_s     = self.get_parameter('timeout_s').value,
            retries       = self.get_parameter('retries').value,
            bit_logic     = self.get_parameter('bit_logic').value,
            invert_error  = self.get_parameter('invert_error').value,
            fallback_median = self.get_parameter('fallback_median').value,
        )

        self._connected  = False
        self._output_mode = 'response'   # mode aktif saat ini

        # ─── Logger ────────────────────────────────────────
        self.logger_csv = MagneticCSVLogger(self.get_parameter('log_dir').value)

        # ─── State ─────────────────────────────────────────
        self._last_data: dict = None   # hasil read_sensor() terakhir
        self._address   = self.get_parameter('slave_id').value

        # ─── ROS Publishers ────────────────────────────────
        self.pub_status = self.create_publisher(Float32MultiArray, 'magnetic/status',     10)
        self.pub_conn   = self.create_publisher(String,            'magnetic/connection', 10)
        self.pub_ack    = self.create_publisher(String,            'magnetic/ack',        10)
        self.pub_log    = self.create_publisher(String,            'magnetic/log',        10)

        # ─── ROS Subscriber ────────────────────────────────
        self.create_subscription(String, 'magnetic/cmd', self.cmd_cb, 10)

        # ─── Connect ───────────────────────────────────────
        self._do_connect()

        # ─── Timers ────────────────────────────────────────
        poll_period = 1.0 / max(1.0, self.get_parameter('poll_rate_hz').value)
        self.create_timer(poll_period, self.read_and_publish)    # read sensor
        self.create_timer(1.0,         self.publish_connection)  # connection watchdog

        self.get_logger().info(
            f'AMR Magnetic Node ACTIVE — port={self.get_parameter("port").value} '
            f'baud={self.get_parameter("baudrate").value} slave={self._address}'
        )

    # ═══════════════════════════════════════════════════════
    # CONNECTION
    # ═══════════════════════════════════════════════════════

    def _do_connect(self):
        try:
            ok = self.sensor.connect()
            if ok:
                self._connected = True
                self.get_logger().info('Magnetic sensor connected')
                self._log_ui('CONNECT', 'ok', f'Connected port={self.get_parameter("port").value}')
            else:
                self._connected = False
                self.get_logger().error('Magnetic sensor connection failed')
                self._log_ui('CONNECT', 'error', 'Connection failed')
        except Exception as e:
            self._connected = False
            self.get_logger().error(f'Connection error: {e}')

    # ═══════════════════════════════════════════════════════
    # READ SENSOR TIMER
    # ═══════════════════════════════════════════════════════

    def read_and_publish(self):
        if self._shutting_down:
            return

        if not self._connected:
            # Coba reconnect setiap 5 detik
            if not hasattr(self, '_last_reconnect'):
                self._last_reconnect = 0.0
            now = time.time()
            if now - self._last_reconnect > 5.0:
                self._last_reconnect = now
                self.get_logger().warn('Attempting reconnect...')
                ok = self.sensor.reconnect()
                if ok:
                    self._connected = True
                    self.get_logger().info('Reconnected')
                    self._log_ui('RECONNECT', 'ok', 'Reconnected successfully')
            return

        try:
            data = self.sensor.read_sensor()
            if data is None:
                self._connected = False
                self._log_ui('READ', 'error', 'Read failed — disconnected?')
                return

            self._last_data = data
            self._publish_status(data)

        except Exception as e:
            self._connected = False
            self.get_logger().error(f'Read error: {e}')
            self._log_ui('READ', 'error', str(e))

    def _publish_status(self, d: dict):
        """Publish 27 float ke magnetic/status."""
        active_pts = d.get('active_points', [])
        pts_flags  = [1.0 if (i+1) in active_pts else 0.0 for i in range(SENSOR_COUNT)]

        msg = Float32MultiArray()
        msg.data = [
            float(1 if d.get('line_detected') else 0),    # [0]
            float(d.get('median_value', 0.0)),             # [1]
            float(d.get('median_integer', 0)),             # [2]
            float(d.get('median_decimal', 0)),             # [3]
            float(d.get('active_count', 0)),               # [4]
            float(d.get('lateral_error_mm') or 9999.0),   # [5] 9999 = N/A
            float(QUALITY_CODE.get(d.get('quality', 'NO_LINE'), 0)),  # [6]
            float(d.get('position_mask', 0)),              # [7]
            float(self._address),                          # [8]
            float(d.get('raw_reg0', 0)),                   # [9]
            float(d.get('raw_reg1', 0)),                   # [10]
        ] + pts_flags                                      # [11..26] P1~P16

        self.pub_status.publish(msg)

    # ═══════════════════════════════════════════════════════
    # CONNECTION WATCHDOG TIMER
    # ═══════════════════════════════════════════════════════

    def publish_connection(self):
        if self._shutting_down:
            return
        if self._connected:
            self.pub_conn.publish(String(data='OK'))
        else:
            self.pub_conn.publish(String(data='DISCONNECTED'))

    # ═══════════════════════════════════════════════════════
    # COMMAND HANDLER  (magnetic/cmd)
    # ═══════════════════════════════════════════════════════

    def cmd_cb(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._ack(f'ERROR JSON: {e}')
            return

        action = cmd.get('action', '')

        if action == 'read_address':
            self._do_read_address()

        elif action == 'set_address':
            self._do_set_address(cmd)

        elif action == 'set_baudrate':
            self._do_set_baudrate(cmd)

        elif action == 'set_mode':
            self._do_set_mode(cmd)

        elif action == 'set_frequency':
            self._do_set_frequency(cmd)

        else:
            self._ack(f'ERROR unknown action: {action}')

    # ── Command implementations ────────────────────────────

    def _do_read_address(self):
        try:
            addr = self.sensor.read_address_register()
            if addr is not None:
                self._address = addr
                self._ack(f'OK address={addr}')
                self._log_ui('READ_ADDR', 'ok', f'Address terbaca: {addr}')
                self.logger_csv.log('READ_ADDR', 'ok', f'Address: {addr}')
            else:
                self._ack('ERROR read address gagal')
                self._log_ui('READ_ADDR', 'error', 'Gagal membaca address register')
        except Exception as e:
            self._ack(f'ERROR {e}')
            self._log_ui('READ_ADDR', 'error', str(e))

    def _do_set_address(self, cmd: dict):
        try:
            addr = int(cmd.get('address', 1))
            ok   = self.sensor.set_address(addr)
            if ok:
                self._ack('OK')
                self._log_ui('SET_ADDR', 'ok', f'Address diset ke {addr}. Power cycle diperlukan.')
                self.logger_csv.log('SET_ADDR', 'ok', f'Address → {addr}')
                self.get_logger().info(f'Address set to {addr}')
            else:
                self._ack('ERROR set address gagal')
                self._log_ui('SET_ADDR', 'error', f'Gagal set address ke {addr}')
        except ValueError as e:
            self._ack(f'ERROR {e}')
            self._log_ui('SET_ADDR', 'error', str(e))
        except Exception as e:
            self._ack(f'ERROR {e}')
            self._log_ui('SET_ADDR', 'error', str(e))

    def _do_set_baudrate(self, cmd: dict):
        try:
            baud = int(cmd.get('baudrate', 9600))
            ok   = self.sensor.set_baudrate(baud)
            if ok:
                self._ack('OK')
                self._log_ui('SET_BAUD', 'ok', f'Baudrate diset ke {baud}. Power cycle diperlukan.')
                self.logger_csv.log('SET_BAUD', 'ok', f'Baudrate → {baud}')
                self.get_logger().info(f'Baudrate set to {baud}')
            else:
                self._ack('ERROR set baudrate gagal')
                self._log_ui('SET_BAUD', 'error', f'Gagal set baudrate ke {baud}')
        except ValueError as e:
            self._ack(f'ERROR {e}')
            self._log_ui('SET_BAUD', 'error', str(e))
        except Exception as e:
            self._ack(f'ERROR {e}')
            self._log_ui('SET_BAUD', 'error', str(e))

    def _do_set_mode(self, cmd: dict):
        try:
            mode = cmd.get('mode', 'response')
            ok   = self.sensor.set_output_mode(mode)
            if ok:
                self._output_mode = mode
                self._ack('OK')
                self._log_ui('SET_MODE', 'ok', f'Output mode → {mode}. Power cycle diperlukan.')
                self.logger_csv.log('SET_MODE', 'ok', f'Mode → {mode}')
                self.get_logger().info(f'Output mode set to {mode}')
            else:
                self._ack('ERROR set mode gagal')
                self._log_ui('SET_MODE', 'error', f'Gagal set mode ke {mode}')
        except ValueError as e:
            self._ack(f'ERROR {e}')
            self._log_ui('SET_MODE', 'error', str(e))
        except Exception as e:
            self._ack(f'ERROR {e}')
            self._log_ui('SET_MODE', 'error', str(e))

    def _do_set_frequency(self, cmd: dict):
        try:
            freq = int(cmd.get('frequency', 10))
            if self._output_mode != 'continuous':
                self._ack('ERROR frequency hanya berlaku untuk mode continuous')
                self._log_ui('SET_FREQ', 'error', 'Mode bukan continuous')
                return
            ok = self.sensor.set_output_frequency(freq)
            if ok:
                self._ack('OK')
                self._log_ui('SET_FREQ', 'ok', f'Frequency → {freq} Hz. Power cycle diperlukan.')
                self.logger_csv.log('SET_FREQ', 'ok', f'Freq → {freq} Hz')
                self.get_logger().info(f'Frequency set to {freq} Hz')
            else:
                self._ack('ERROR set frequency gagal')
                self._log_ui('SET_FREQ', 'error', f'Gagal set frequency ke {freq} Hz')
        except ValueError as e:
            self._ack(f'ERROR {e}')
            self._log_ui('SET_FREQ', 'error', str(e))
        except Exception as e:
            self._ack(f'ERROR {e}')
            self._log_ui('SET_FREQ', 'error', str(e))

    # ═══════════════════════════════════════════════════════
    # HELPERS
    # ═══════════════════════════════════════════════════════

    def _ack(self, msg: str):
        self.pub_ack.publish(String(data=msg))

    def _log_ui(self, tag: str, level: str, msg: str):
        """Publish log entry ke magnetic/log (JSON) untuk web UI."""
        ts = datetime.now().isoformat(timespec='microseconds')
        ts_display = ts.replace('T', ' ')[:19]
        entry = json.dumps({
            'ts':         ts,
            'ts_display': ts_display,
            'tag':        tag,
            'level':      level,
            'msg':        msg,
        })
        self.pub_log.publish(String(data=entry))
        self.logger_csv.log(tag, level, msg)


# ═══════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = AMRMagneticNode()

    def shutdown_hook():
        node.get_logger().warn('ROS shutdown → close sensor')
        node._shutting_down = True
        try:
            node.sensor.close()
        except Exception:
            pass

    rclpy.get_default_context().on_shutdown(shutdown_hook)

    try:
        rclpy.spin(node)
    finally:
        node.logger_csv.close()
        node.destroy_node()