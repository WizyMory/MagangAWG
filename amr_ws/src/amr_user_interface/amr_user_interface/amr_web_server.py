#!/usr/bin/env python3
"""
FastAPI Bridge – AMR 25 Teknisi Kontrol
PT. Adhikara Wiyasa Gani

Update v2:
  - BLDCBridgeNode: subscribe bldc/status (13 float), bldc/connection, bldc/ack
  - /api/bldc/status: return semua 13 field termasuk status_word, torque, driver_temp, bus_voltage
  - /api/bldc/cmd: terima JSON payload dari frontend dan publish ke bldc/cmd
  - /api/bldc/logs & /api/bldc/logs/full: baca CSV log dari amr_motor node
  - read_bldc_logs: parse CSV format (time, mode, action, result)
  - _ros_spin_executor: MultiThreadedExecutor untuk relay + bldc node
"""

import json
import threading
import time
import os
import csv
from contextlib import asynccontextmanager
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from ament_index_python.packages import get_package_share_directory


# ═══════════════════════════════════════════════════════════
# RELAY BRIDGE NODE
# ═══════════════════════════════════════════════════════════

class RelayBridgeNode(Node):
    def __init__(self):
        super().__init__('relay_bridge_node')

        self.relay_state:  list  = [0] * 8
        self.modbus_ok:    bool  = False
        self.modbus_fault: bool  = False
        self.last_ack:     str   = ''
        self.ack_event           = threading.Event()
        self._last_conn_ts: float = 0.0

        self.create_subscription(Int32MultiArray, 'relay/status',     self._on_status,     10)
        self.create_subscription(String,          'relay/ack',        self._on_ack,        10)
        self.create_subscription(String,          'relay/connection', self._on_connection, 10)

        self.pub_cmd = self.create_publisher(String, 'relay/cmd', 10)
        self.create_timer(1.0, self._check_connection)

        self.get_logger().info('RelayBridgeNode ACTIVE')

    def _on_connection(self, msg: String):
        self._last_conn_ts = time.time()
        if msg.data == 'OK':
            self.modbus_ok    = True
            self.modbus_fault = False
        elif msg.data == 'FAULT':
            self.modbus_ok    = False
            self.modbus_fault = True
        else:
            self.modbus_ok    = False
            self.modbus_fault = False

    def _on_status(self, msg: Int32MultiArray):
        self.relay_state = list(msg.data)

    def _on_ack(self, msg: String):
        self.last_ack = msg.data
        if msg.data.startswith('ERROR'):
            self.modbus_fault = True
        self.ack_event.set()

    def _check_connection(self):
        if time.time() - self._last_conn_ts > 5.0:
            self.modbus_ok    = False
            self.modbus_fault = False

    def publish_command(self, command: str) -> str:
        self.ack_event.clear()
        self.pub_cmd.publish(String(data=command))
        got = self.ack_event.wait(timeout=3.0)
        return self.last_ack if got else 'ERROR timeout'


# ═══════════════════════════════════════════════════════════
# BLDC BRIDGE NODE
# Subscribe ke topic dari amr_motor node
# ═══════════════════════════════════════════════════════════

class BLDCBridgeNode(Node):
    """
    Bridge node untuk BLDC Motor Driver ZLAC8015D.

    Topics yang di-subscribe (dari amr_motor node):
      bldc/status      → Float32MultiArray
        data[0]  = vel_actual_l    (0.1 r/min)
        data[1]  = vel_actual_r    (0.1 r/min)
        data[2]  = pos_actual_l    (counts, int32 as float)
        data[3]  = pos_actual_r    (counts, int32 as float)
        data[4]  = torque_actual_l (0.1 A raw)
        data[5]  = torque_actual_r (0.1 A raw)
        data[6]  = motor_temp_l    (°C)
        data[7]  = motor_temp_r    (°C)
        data[8]  = driver_temp     (0.1 °C raw)
        data[9]  = bus_voltage     (0.01 V raw)
        data[10] = status_word     (register 0x20A2)
        data[11] = error_l         (register 0x20A5)
        data[12] = error_r         (register 0x20A6)

      bldc/connection  → String  ("OK" | "FAULT" | "DISCONNECTED")
      bldc/ack         → String  ("OK" | "ERROR ...")

    Topics yang di-publish (ke amr_motor node):
      bldc/cmd         → String (JSON command)
    """

    def __init__(self):
        super().__init__('bldc_bridge_node')

        # ── Status state ──
        self.connected:       bool  = False
        self.vel_actual_l:    float = 0.0
        self.vel_actual_r:    float = 0.0
        self.pos_actual_l:    int   = 0
        self.pos_actual_r:    int   = 0
        self.torque_actual_l: float = 0.0
        self.torque_actual_r: float = 0.0
        self.motor_temp_l:    int   = 0
        self.motor_temp_r:    int   = 0
        self.driver_temp:     float = 0.0   # raw 0.1°C
        self.bus_voltage:     float = 0.0   # raw 0.01V
        self.status_word:     int   = 0
        self.error_l:         int   = 0
        self.error_r:         int   = 0

        self._last_conn_ts: float = 0.0
        self.last_ack:      str   = ''
        self.ack_event            = threading.Event()

        # ── Subscriptions ──
        self.create_subscription(
            Float32MultiArray, 'bldc/status',     self._on_status,     10)
        self.create_subscription(
            String,            'bldc/connection', self._on_connection, 10)
        self.create_subscription(
            String,            'bldc/ack',        self._on_ack,        10)

        # ── Publisher ──
        self.pub_cmd = self.create_publisher(String, 'bldc/cmd', 10)

        # ── Connectivity watchdog ──
        self.create_timer(1.0, self._check_connection)

        self.get_logger().info('BLDCBridgeNode ACTIVE')

    def _on_status(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) >= 13:
            self.vel_actual_l    = d[0]
            self.vel_actual_r    = d[1]
            self.pos_actual_l    = int(d[2])
            self.pos_actual_r    = int(d[3])
            self.torque_actual_l = d[4]
            self.torque_actual_r = d[5]
            self.motor_temp_l    = int(d[6])
            self.motor_temp_r    = int(d[7])
            self.driver_temp     = d[8]    # 0.1°C raw
            self.bus_voltage     = d[9]    # 0.01V raw
            self.status_word     = int(d[10])
            self.error_l         = int(d[11])
            self.error_r         = int(d[12])
        self._last_conn_ts = time.time()

    def _on_connection(self, msg: String):
        self._last_conn_ts = time.time()
        self.connected = (msg.data == 'OK')

    def _on_ack(self, msg: String):
        self.last_ack = msg.data
        self.ack_event.set()

    def _check_connection(self):
        if time.time() - self._last_conn_ts > 5.0:
            self.connected = False

    def publish_command(self, command: str) -> str:
        """Publish JSON command string dan tunggu ACK (timeout 3 detik)."""
        self.ack_event.clear()
        self.pub_cmd.publish(String(data=command))
        got = self.ack_event.wait(timeout=3.0)
        return self.last_ack if got else 'ERROR timeout'



# ═══════════════════════════════════════════════════════════
# MAGNETIC BRIDGE NODE
# Subscribe ke topic dari amr_magnetic_node
# ═══════════════════════════════════════════════════════════

class MagneticBridgeNode(Node):
    """
    Bridge node untuk CCF-NS16 Magnetic Line Sensor.

    Topics yang di-subscribe (dari amr_magnetic_node):
      magnetic/status     → Float32MultiArray [27 float]
        [0]   line_detected    (0/1)
        [1]   median_value     (float)
        [2]   median_integer   (int)
        [3]   median_decimal   (int)
        [4]   active_count     (int)
        [5]   lateral_error_mm (float, 9999.0 = N/A)
        [6]   quality_code     (0=NO_LINE,1=GOOD,2=WIDE_LINE,3=TOO_MANY_POINTS)
        [7]   position_mask    (int)
        [8]   address          (int)
        [9]   raw_reg0         (int)
        [10]  raw_reg1         (int)
        [11..26] P1~P16 active flags (1.0 = aktif, 0.0 = tidak)

      magnetic/connection → String ("OK" | "DISCONNECTED")
      magnetic/ack        → String ("OK" | "ERROR ...")
      magnetic/log        → String (JSON log entry dari node)

    Topics yang di-publish (ke amr_magnetic_node):
      magnetic/cmd        → String (JSON command)
    """

    QUALITY_NAMES = {0: 'NO_LINE', 1: 'GOOD', 2: 'WIDE_LINE', 3: 'TOO_MANY_POINTS'}

    def __init__(self):
        super().__init__('magnetic_bridge_node')

        # ── State ──
        self.connected:        bool        = False
        self.line_detected:    bool        = False
        self.median_value:     float       = 0.0
        self.median_integer:   int         = 0
        self.median_decimal:   int         = 0
        self.active_count:     int         = 0
        self.lateral_error_mm: float       = 9999.0
        self.quality:          str         = 'NO_LINE'
        self.position_mask:    int         = 0
        self.address:          int         = 1
        self.raw_reg0:         int         = 0
        self.raw_reg1:         int         = 0
        self.active_points:    list        = []   # list of 1-based point numbers
        self.output_mode:      str         = 'response'

        self._last_conn_ts: float = 0.0
        self.last_ack:      str   = ''
        self.ack_event            = threading.Event()

        # Log buffer — simpan N entry terakhir dari magnetic/log
        self._log_buffer: list = []
        self._log_max    = 500

        # ── Subscriptions ──
        self.create_subscription(
            Float32MultiArray, 'magnetic/status',     self._on_status,     10)
        self.create_subscription(
            String,            'magnetic/connection', self._on_connection, 10)
        self.create_subscription(
            String,            'magnetic/ack',        self._on_ack,        10)
        self.create_subscription(
            String,            'magnetic/log',        self._on_log,        10)

        # ── Publisher ──
        self.pub_cmd = self.create_publisher(String, 'magnetic/cmd', 10)

        # ── Watchdog ──
        self.create_timer(1.0, self._check_connection)

        self.get_logger().info('MagneticBridgeNode ACTIVE')

    def _on_status(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) < 27:
            return
        self.line_detected    = bool(d[0])
        self.median_value     = float(d[1])
        self.median_integer   = int(d[2])
        self.median_decimal   = int(d[3])
        self.active_count     = int(d[4])
        self.lateral_error_mm = float(d[5])
        self.quality          = self.QUALITY_NAMES.get(int(d[6]), 'UNKNOWN')
        self.position_mask    = int(d[7])
        self.address          = int(d[8])
        self.raw_reg0         = int(d[9])
        self.raw_reg1         = int(d[10])
        # Active points: P1~P16 (index 11..26)
        self.active_points    = [i + 1 for i in range(16) if d[11 + i] == 1.0]
        self._last_conn_ts    = time.time()

    def _on_connection(self, msg: String):
        self._last_conn_ts = time.time()
        self.connected = (msg.data == 'OK')

    def _on_ack(self, msg: String):
        self.last_ack = msg.data
        self.ack_event.set()

    def _on_log(self, msg: String):
        """Terima log entry JSON dari amr_magnetic_node, simpan ke buffer."""
        try:
            entry = json.loads(msg.data)
            self._log_buffer.append(entry)
            if len(self._log_buffer) > self._log_max:
                self._log_buffer = self._log_buffer[-self._log_max:]
        except Exception:
            pass

    def _check_connection(self):
        if time.time() - self._last_conn_ts > 5.0:
            self.connected = False

    def publish_command(self, command: str) -> str:
        """Publish JSON command ke magnetic/cmd dan tunggu ACK."""
        self.ack_event.clear()
        self.pub_cmd.publish(String(data=command))
        got = self.ack_event.wait(timeout=3.0)
        return self.last_ack if got else 'ERROR timeout'

LOG_DIR = os.path.expanduser('~/amr_ws/src/amr_relay/logs')

def read_relay_logs(limit: Optional[int] = None) -> list:
    entries = []
    if not os.path.isdir(LOG_DIR):
        return entries

    for fname in sorted(os.listdir(LOG_DIR), reverse=True):
        if not fname.endswith('.csv'):
            continue
        fpath = os.path.join(LOG_DIR, fname)
        try:
            with open(fpath, newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    ts_raw     = row.get('time', '')
                    ts_display = ts_raw.replace('T', ' ')[:19] if ts_raw else ''
                    entries.append({
                        'ts':         ts_raw,
                        'ts_display': ts_display,
                        'channel':    int(row.get('channel', -1)),
                        'state':      row.get('state', 'OFF').upper(),
                        'note':       row.get('result', ''),
                    })
        except Exception:
            pass

    entries.sort(key=lambda x: x['ts'], reverse=True)
    return entries[:limit] if limit else entries


# ═══════════════════════════════════════════════════════════
# LOG READER – BLDC
# ═══════════════════════════════════════════════════════════

BLDC_LOG_DIR = os.path.expanduser('~/amr_ws/src/amr_motor/logs')

def read_bldc_logs(limit: Optional[int] = None) -> list:
    """
    Baca CSV log dari amr_motor node.
    Expected CSV header: time, mode, action, result
    """
    entries = []
    if not os.path.isdir(BLDC_LOG_DIR):
        return entries

    for fname in sorted(os.listdir(BLDC_LOG_DIR), reverse=True):
        if not fname.endswith('.csv'):
            continue
        fpath = os.path.join(BLDC_LOG_DIR, fname)
        try:
            with open(fpath, newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    ts_raw     = row.get('time', '')
                    ts_display = ts_raw.replace('T', ' ')[:19] if ts_raw else ''
                    action     = row.get('action', '')
                    result     = row.get('result', '')
                    entries.append({
                        'ts':         ts_raw,
                        'ts_display': ts_display,
                        'tag':        row.get('mode', 'BLDC').upper(),
                        'msg':        f'{action} → {result}' if action else result,
                        'note':       result,
                    })
        except Exception:
            pass

    entries.sort(key=lambda x: x['ts'], reverse=True)
    return entries[:limit] if limit else entries


# ═══════════════════════════════════════════════════════════
# LOG READER – Magnetic Sensor (dari CSV file)
# ═══════════════════════════════════════════════════════════

MAG_LOG_DIR = os.path.expanduser('~/amr_ws/src/amr_front_magnetic/logs')

def read_magnetic_logs(limit: Optional[int] = None) -> list:
    """
    Baca CSV log dari amr_magnetic_node.
    CSV header: time, tag, level, msg
    """
    entries = []
    if not os.path.isdir(MAG_LOG_DIR):
        return entries

    for fname in sorted(os.listdir(MAG_LOG_DIR), reverse=True):
        if not fname.endswith('.csv'):
            continue
        fpath = os.path.join(MAG_LOG_DIR, fname)
        try:
            with open(fpath, newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    ts_raw     = row.get('time', '')
                    ts_display = ts_raw.replace('T', ' ')[:19] if ts_raw else ''
                    entries.append({
                        'ts':         ts_raw,
                        'ts_display': ts_display,
                        'tag':        row.get('tag', 'MAG').upper(),
                        'level':      row.get('level', 'info').lower(),
                        'msg':        row.get('msg', ''),
                    })
        except Exception:
            pass

    entries.sort(key=lambda x: x['ts'], reverse=True)
    return entries[:limit] if limit else entries

relay_node:    Optional[RelayBridgeNode]    = None
bldc_node:     Optional[BLDCBridgeNode]     = None
magnetic_node: Optional[MagneticBridgeNode] = None

def _ros_spin_executor():
    """
    Spin semua node dalam 1 MultiThreadedExecutor.
    Menyelesaikan 'ValueError: generator already executing'.
    """
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(relay_node)
    executor.add_node(bldc_node)
    executor.add_node(magnetic_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()

@asynccontextmanager
async def lifespan(app: FastAPI):
    global relay_node, bldc_node, magnetic_node

    rclpy.init()

    relay_node    = RelayBridgeNode()
    bldc_node     = BLDCBridgeNode()
    magnetic_node = MagneticBridgeNode()

    threading.Thread(target=_ros_spin_executor, daemon=True).start()

    yield

    relay_node.destroy_node()
    bldc_node.destroy_node()
    magnetic_node.destroy_node()
    rclpy.shutdown()


# ═══════════════════════════════════════════════════════════
# FASTAPI APP
# ═══════════════════════════════════════════════════════════

app = FastAPI(title='AMR 25 API Bridge', lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ── REQUEST MODELS ──────────────────────────────────────────

class RelayCmd(BaseModel):
    command: str

class BLDCCmd(BaseModel):
    """
    Contoh payload:

    Velocity start:
      { "mode":"velocity", "action":"start",
        "acc_l":500, "acc_r":500, "dcc_l":500, "dcc_r":500,
        "target_l":100, "target_r":100 }

    Position start:
      { "mode":"position", "action":"start",
        "sync":true, "absolute":false,
        "acc_l":500, "acc_r":500, "dcc_l":500, "dcc_r":500,
        "target_l":20480, "target_r":20480,
        "speed_l":50, "speed_r":50 }

    Torque start:
      { "mode":"torque", "action":"start",
        "slope_l":300, "slope_r":300,
        "target_l":2000, "target_r":2000 }

    Stop / E-stop / Clear / Reset:
      { "action":"stop" }
      { "action":"emergency_stop" }
      { "action":"clear_error" }
      { "action":"reset_data" }
    """
    mode:      Optional[str] = None   # velocity | position | torque
    action:    str           = 'stop'
    # velocity params
    acc_l:     Optional[int] = None
    acc_r:     Optional[int] = None
    dcc_l:     Optional[int] = None
    dcc_r:     Optional[int] = None
    target_l:  Optional[int] = None
    target_r:  Optional[int] = None
    # position extra
    sync:      Optional[bool] = True
    absolute:  Optional[bool] = False
    speed_l:   Optional[int]  = None
    speed_r:   Optional[int]  = None
    # torque extra
    slope_l:   Optional[int]  = None
    slope_r:   Optional[int]  = None


# ── RELAY ENDPOINTS ─────────────────────────────────────────

@app.get('/api/module_status')
def get_module_status():
    relay_status   = 'connected' if relay_node.modbus_ok    else 'disconnected'
    bldc_status    = 'connected' if bldc_node.connected     else 'disconnected'
    mag_status     = 'connected' if magnetic_node.connected else 'disconnected'
    return {
        'modbus_rtu_relay':      relay_status,
        'bldc_motor_driver':     bldc_status,
        'front_magnetic_sensor': mag_status,
        'lidar':                 'disconnected',
        'rear_magnetic_sensor':  'disconnected',
        'imu_sensor':            'disconnected',
        'camera_module':         'disconnected',
        'emergency_stop':        'disconnected',
    }

@app.get('/api/relay/status')
def get_relay_status():
    return {
        'connected':    relay_node.modbus_ok,
        'modbus_fault': relay_node.modbus_fault,
        'relay_state':  relay_node.relay_state,
    }

@app.post('/api/relay/cmd')
def post_relay_cmd(body: RelayCmd):
    ack = relay_node.publish_command(body.command.strip())
    return {'ack': ack}

@app.get('/api/relay/logs')
def get_relay_logs(limit: int = 8):
    return read_relay_logs(limit=limit)

@app.get('/api/relay/logs/full')
def get_relay_logs_full():
    return read_relay_logs()


# ── BLDC ENDPOINTS ──────────────────────────────────────────

@app.get('/api/bldc/status')
def get_bldc_status():
    """
    Return semua data monitoring BLDC motor driver.
    Frontend bldc.html polling endpoint ini setiap 500ms.

    Catatan konversi untuk frontend:
      vel_actual_l/r  : raw 0.1 r/min  → tampil langsung (satuan di label)
      torque_actual_l/r: raw 0.1 A     → tampil langsung
      driver_temp     : raw 0.1°C      → kalikan 0.1 untuk dapat °C
      bus_voltage     : raw 0.01V      → kalikan 0.01 untuk dapat Volt
      motor_temp_l/r  : sudah °C       → tampil langsung
      status_word     : integer hex    → decode bits [7:6] untuk drive state
      error_l/r       : integer bitmask→ decode per ERROR_BITS
    """
    return {
        'connected':       bldc_node.connected,
        'vel_actual_l':    bldc_node.vel_actual_l,
        'vel_actual_r':    bldc_node.vel_actual_r,
        'pos_actual_l':    bldc_node.pos_actual_l,
        'pos_actual_r':    bldc_node.pos_actual_r,
        'torque_actual_l': bldc_node.torque_actual_l,
        'torque_actual_r': bldc_node.torque_actual_r,
        'motor_temp_l':    bldc_node.motor_temp_l,
        'motor_temp_r':    bldc_node.motor_temp_r,
        'driver_temp':     bldc_node.driver_temp,    # 0.1°C raw
        'bus_voltage':     bldc_node.bus_voltage,    # 0.01V raw
        'status_word':     bldc_node.status_word,
        'error_l':         bldc_node.error_l,
        'error_r':         bldc_node.error_r,
    }

@app.post('/api/bldc/cmd')
def post_bldc_cmd(body: BLDCCmd):
    """
    Terima command dari frontend dan publish ke topic bldc/cmd.
    amr_motor node akan parse JSON dan mengirim Modbus RTU ke ZLAC8015D.

    Action yang didukung:
      mode=velocity, action=start  → _do_velocity_start
      mode=position, action=start  → _do_position_start
      mode=torque,   action=start  → _do_torque_start
      action=stop                  → _do_stop
      action=emergency_stop        → _do_emergency_stop
      action=clear_error           → _do_clear_error
      action=reset_data            → _do_reset_data
    """
    import json
    payload = body.model_dump(exclude_none=True)
    cmd_str = json.dumps(payload)
    ack = bldc_node.publish_command(cmd_str)
    return {'ack': ack, 'status': 'sent', 'payload': payload}

@app.get('/api/bldc/logs')
def get_bldc_logs(limit: int = 30):
    return read_bldc_logs(limit=limit)

@app.get('/api/bldc/logs/full')
def get_bldc_logs_full():
    return read_bldc_logs()


# ── MAGNETIC ENDPOINTS ──────────────────────────────────────

class MagneticCmd(BaseModel):
    """
    Command dari magnetic.html ke amr_magnetic_node via magnetic/cmd.

    { "action": "set_address",   "address": 1 }
    { "action": "set_baudrate",  "baudrate": 9600 }
    { "action": "set_mode",      "mode": "response" | "continuous" | "change" }
    { "action": "set_frequency", "frequency": 10 | 25 | 50 | 100 }
    { "action": "read_address" }
    """
    action:    str            = 'read_address'
    address:   Optional[int]  = None
    baudrate:  Optional[int]  = None
    mode:      Optional[str]  = None
    frequency: Optional[int]  = None

@app.get('/api/magnetic/status')
def get_magnetic_status():
    """
    Return semua data sensor magnetic.
    Frontend magnetic.html polling endpoint ini setiap 300ms.
    """
    m = magnetic_node
    return {
        'connected':        m.connected,
        'line_detected':    m.line_detected,
        'median_value':     m.median_value,
        'median_integer':   m.median_integer,
        'median_decimal':   m.median_decimal,
        'active_count':     m.active_count,
        'lateral_error_mm': None if m.lateral_error_mm == 9999.0 else m.lateral_error_mm,
        'quality':          m.quality,
        'position_mask':    m.position_mask,
        'address':          m.address,
        'raw_reg0':         m.raw_reg0,
        'raw_reg1':         m.raw_reg1,
        'active_points':    m.active_points,   # list of 1-based point numbers
        'output_mode':      m.output_mode,
    }

@app.post('/api/magnetic/cmd')
def post_magnetic_cmd(body: MagneticCmd):
    """
    Forward command ke amr_magnetic_node via magnetic/cmd topic.
    """
    payload = body.model_dump(exclude_none=True)
    cmd_str = json.dumps(payload)
    ack = magnetic_node.publish_command(cmd_str)
    return {'ack': ack, 'payload': payload}

@app.get('/api/magnetic/logs')
def get_magnetic_logs(limit: int = 20):
    """
    Gabungan: log terbaru dari buffer in-memory (magnetic/log topic)
    + log dari CSV file. In-memory lebih segar, CSV lebih persisten.
    """
    # Ambil dari buffer in-memory (dari magnetic/log topic)
    mem_logs = list(reversed(magnetic_node._log_buffer[-limit:]))

    # Fallback ke CSV jika buffer kosong
    if not mem_logs:
        return read_magnetic_logs(limit=limit)

    return mem_logs[:limit]

@app.get('/api/magnetic/logs/full')
def get_magnetic_logs_full():
    """Semua log dari CSV untuk halaman mag_log.html."""
    return read_magnetic_logs()


# ── STATIC FILES ── paling bawah ────────────────────────────

STATIC_DIR = os.path.join(
    get_package_share_directory('amr_user_interface'), 'static'
)
app.mount('/', StaticFiles(directory=STATIC_DIR, html=True), name='static')


# ── ENTRY POINT ─────────────────────────────────────────────

def main():
    import uvicorn
    uvicorn.run(app, host='0.0.0.0', port=8000)

if __name__ == '__main__':
    main()