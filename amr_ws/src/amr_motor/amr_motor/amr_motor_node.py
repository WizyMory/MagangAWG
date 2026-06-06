#!/usr/bin/env python3
"""
AMR Motor Node – ZLAC8015D
PT. Adhikara Wiyasa Gani

Fix v2:
  - Tambah flag _ui_controlled untuk memisahkan kontrol UI vs nav stack (cmd_vel)
  - control_loop SKIP sepenuhnya saat _ui_controlled = True
    → mencegah override target_rpm menjadi 0 setelah UI kirim velocity
  - modbus_write_loop juga skip saat _ui_controlled = True
    (driver sudah ditulis langsung via write_velocity, tidak perlu loop ini)
  - _do_stop / _do_emergency_stop reset _ui_controlled = False
    sehingga nav stack bisa kembali kontrol setelah stop
  - enable_srv_cb (nav stack) juga set _ui_controlled = False

Fix v3:
  - Tambah handler reset_data di ui_cmd_cb
  - _do_reset_data: reset semua state monitoring ke 0
    (berguna untuk membersihkan data stale setelah driver restart)

Fix v4:
  - Baca status_word (0x20A2) dan torque actual di timer masing-masing
  - publish_bldc_status menyertakan status_word, torque_actual_l/r, driver_temp, bus_voltage
    → semua 13 field sudah benar dikirim ke BLDCBridgeNode di web server
  - Tambah read_status_word_timer (1 Hz)
  - Tambah read_torque_timer (1 Hz)
  - Driver temp raw = 0.1°C, bus voltage raw = 0.01V (sudah benar)
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import SetBool, Trigger

from tf2_ros import TransformBroadcaster
from diagnostic_updater import Updater, DiagnosticStatusWrapper

import math
import json
import os
import csv
import threading
from datetime import datetime

from amr_motor.zlac8015d import ZLAC8015D
from amr_motor.diff_drive import twist_to_rpm


# ═══════════════════════════════════════════════════════════
# LOGGER CSV
# ═══════════════════════════════════════════════════════════

class MotorCSVLogger:
    """
    Menulis log aktivitas motor ke CSV.
    Format: time, mode, action, result
    """
    def __init__(self, log_dir: str):
        os.makedirs(log_dir, exist_ok=True)
        fname = datetime.now().strftime('%Y-%m-%d') + '_motor.csv'
        self._path = os.path.join(log_dir, fname)
        self._lock = threading.Lock()

        if not os.path.exists(self._path):
            with open(self._path, 'w', newline='') as f:
                csv.writer(f).writerow(['time', 'mode', 'action', 'result'])

    def log(self, mode: str, action: str, result: str):
        ts = datetime.now().isoformat(timespec='microseconds')
        with self._lock:
            try:
                with open(self._path, 'a', newline='') as f:
                    csv.writer(f).writerow([ts, mode, action, result])
            except Exception:
                pass

    def close(self):
        pass


# ═══════════════════════════════════════════════════════════
# AMR MOTOR NODE
# ═══════════════════════════════════════════════════════════

class AMRMotorNode(Node):
    """
    AMR MOTOR NODE – DRIVER ACCEL + ENCODER ODOMETRY + WEB UI BRIDGE

    Dua sumber perintah motor:
      A) Nav stack  → /cmd_vel → control_loop → modbus_write_loop
      B) Web UI     → bldc/cmd → _do_velocity_start / position / torque

    Flag _ui_controlled:
      False = nav stack (cmd_vel) yang kontrol (default)
      True  = web UI yang kontrol
        → control_loop dan modbus_write_loop di-skip total
        → driver ditulis langsung oleh handler UI

    Transisi ke False kembali saat:
      - _do_stop()
      - _do_emergency_stop()
      - enable_srv_cb() dipanggil dari nav stack

    Topics yang dipublish ke web UI:
      bldc/status      Float32MultiArray  → data monitoring real-time (13 float)
      bldc/connection  String             → "OK" | "FAULT" | "DISCONNECTED"
      bldc/ack         String             → "OK" | "ERROR ..."

    Topic yang di-subscribe dari web UI:
      bldc/cmd         String (JSON)

    Format bldc/status (13 float):
      [0]  vel_actual_l    (0.1 r/min)
      [1]  vel_actual_r    (0.1 r/min)
      [2]  pos_actual_l    (counts, int32 as float)
      [3]  pos_actual_r    (counts, int32 as float)
      [4]  torque_actual_l (0.1 A)
      [5]  torque_actual_r (0.1 A)
      [6]  motor_temp_l    (°C)
      [7]  motor_temp_r    (°C)
      [8]  driver_temp     (0.1 °C raw)
      [9]  bus_voltage     (0.01 V raw)
      [10] status_word     (0x20A2)
      [11] error_l         (0x20A5)
      [12] error_r         (0x20A6)

    Format bldc/cmd (JSON string):
      Velocity:
        {"mode":"velocity","action":"start",
         "acc_l":500,"acc_r":500,"dcc_l":500,"dcc_r":500,
         "target_l":100,"target_r":100}
      Position:
        {"mode":"position","action":"start",
         "sync":true,"absolute":false,
         "acc_l":500,"acc_r":500,"dcc_l":500,"dcc_r":500,
         "target_l":20480,"target_r":20480,"speed_l":50,"speed_r":50}
      Torque:
        {"mode":"torque","action":"start",
         "slope_l":300,"slope_r":300,"target_l":2000,"target_r":2000}
      Control:
        {"action":"stop"}
        {"action":"emergency_stop"}
        {"action":"clear_error"}
        {"action":"reset_data"}
    """

    def __init__(self):
        super().__init__('amr_motor')

        # ─── Shutdown flag ─────────────────────────────────
        self._shutting_down = False

        # ─── Parameters ────────────────────────────────────
        self.declare_parameters('', [
            ('port',            '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0002T4J-if00-port0'),
            ('baudrate',        115200),
            ('slave_id',        1),
            ('wheel_radius',    0.065),
            ('wheel_base',      0.3118),
            ('encoder_cpr',     4096),
            ('gear_ratio',      1.0),
            ('cmd_vel_timeout', 0.5),
            ('max_rpm',         3000),
            ('accel_ms',        1000),
            ('decel_ms',        800),
            ('quick_stop_ms',   10),
            ('log_dir',         os.path.expanduser('~/amr_ws/src/amr_motor/logs')),
        ])

        # ─── Driver ────────────────────────────────────────
        self.driver = ZLAC8015D(
            port=self.get_parameter('port').value,
            baudrate=self.get_parameter('baudrate').value,
            slave_id=self.get_parameter('slave_id').value,
        )
        self.driver.connect()

        # ─── Control source flag ───────────────────────────
        # False = nav stack (cmd_vel) yang kontrol
        # True  = web UI yang kontrol (velocity/position/torque dari bldc/cmd)
        self._ui_controlled = False

        # ─── Mode tracking ─────────────────────────────────
        self._ui_mode   = 'velocity'
        self._driver_ok = False
        self._init_driver()

        # ─── Logger ────────────────────────────────────────
        self.logger_csv = MotorCSVLogger(self.get_parameter('log_dir').value)

        # ─── State: navigation ─────────────────────────────
        self.motor_enabled    = False
        self.cmd              = Twist()
        self.last_cmd_time    = self.get_clock().now()
        self.cmd_vel_received = False

        self.target_rpm_l = 0
        self.target_rpm_r = 0
        self.sent_rpm_l   = 0
        self.sent_rpm_r   = 0

        # ─── State: odometry ───────────────────────────────
        self.enc_l          = None
        self.enc_r          = None
        self.prev_enc_l     = None
        self.prev_enc_r     = None
        self.x = self.y     = 0.0
        self.yaw            = 0.0
        self.last_odom_time = self.get_clock().now()

        # ─── State: monitoring ─────────────────────────────
        self.vel_actual_l    = 0.0
        self.vel_actual_r    = 0.0
        self.pos_actual_l    = 0
        self.pos_actual_r    = 0
        self.torque_actual_l = 0.0   # 0.1 A raw
        self.torque_actual_r = 0.0   # 0.1 A raw
        self.motor_temp_l    = 0
        self.motor_temp_r    = 0
        self.driver_temp_raw = 0      # raw 0.1°C
        self.bus_voltage_raw = 0      # raw 0.01V
        self.status_word     = 0      # register 0x20A2
        self.error_code_l    = 0
        self.error_code_r    = 0

        # ─── ROS Interface ─────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_service(SetBool, '/amr_motor/enable',         self.enable_srv_cb)
        self.create_service(Trigger, '/amr_motor/reset_odometry', self.reset_odometry_cb)

        self.odom_pub       = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Web UI bridge
        self.create_subscription(String, 'bldc/cmd', self.ui_cmd_cb, 10)

        self.pub_status = self.create_publisher(Float32MultiArray, 'bldc/status',     10)
        self.pub_conn   = self.create_publisher(String,            'bldc/connection', 10)
        self.pub_ack    = self.create_publisher(String,            'bldc/ack',        10)

        # ─── Timers ────────────────────────────────────────
        self.create_timer(0.02,  self.control_loop)           # 50 Hz  – komputasi RPM (nav stack)
        self.create_timer(0.05,  self.modbus_write_loop)      # 20 Hz  – kirim RPM ke driver
        self.create_timer(0.05,  self.read_motion_state)      # 20 Hz  – baca vel + encoder
        self.create_timer(0.05,  self.update_odometry)        # 20 Hz  – odometry
        self.create_timer(0.5,   self.publish_bldc_status)    # 2 Hz   – publish ke web UI
        self.create_timer(1.0,   self.publish_connection)     # 1 Hz   – publish koneksi
        self.create_timer(1.0,   self.read_error_code_timer)  # 1 Hz   – error code
        self.create_timer(1.0,   self.read_status_word_timer) # 1 Hz   – status word (NEW v4)
        self.create_timer(1.0,   self.read_torque_timer)      # 1 Hz   – torque actual (NEW v4)
        self.create_timer(2.0,   self.read_voltage_timer)     # 0.5 Hz – bus voltage
        self.create_timer(2.0,   self.read_temp_timer)        # 0.5 Hz – temperatur

        # ─── Diagnostics ───────────────────────────────────
        self.updater = Updater(self)
        self.updater.setHardwareID('ZLAC8015D')
        self.updater.add('Motor Driver', self.diagnostic_cb)
        self.create_timer(1.0, self.updater.update)

        self.get_logger().info('AMR Motor ready (DISABLED)')

    # ═══════════════════════════════════════════════════════
    # DRIVER INIT
    # ═══════════════════════════════════════════════════════

    def _init_driver(self):
        """Inisialisasi driver ke velocity mode (default)."""
        try:
            self.driver.set_velocity_mode()
            self.driver.set_accel_profile(
                accel_ms=self.get_parameter('accel_ms').value,
                decel_ms=self.get_parameter('decel_ms').value,
                quick_stop_ms=self.get_parameter('quick_stop_ms').value,
            )
            self.driver.stop()
            self._driver_ok = True
            self.get_logger().info(
                f'ZLAC init OK – accel={self.get_parameter("accel_ms").value} ms, '
                f'decel={self.get_parameter("decel_ms").value} ms'
            )
        except Exception as e:
            self._driver_ok = False
            self.get_logger().error(f'ZLAC init FAILED: {e}')

    # ═══════════════════════════════════════════════════════
    # WEB UI COMMAND HANDLER  (bldc/cmd)
    # ═══════════════════════════════════════════════════════

    def ui_cmd_cb(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._ack(f'ERROR JSON parse: {e}')
            return

        action = cmd.get('action', 'stop')

        if action == 'emergency_stop':
            self._do_emergency_stop()
            return

        if action == 'clear_error':
            self._do_clear_error()
            return

        if action == 'stop':
            self._do_stop()
            return

        # ── reset_data: bersihkan semua state monitoring ──
        if action == 'reset_data':
            self._do_reset_data()
            return

        mode = cmd.get('mode', 'velocity')

        if mode == 'velocity' and action == 'start':
            self._do_velocity_start(cmd)
        elif mode == 'position' and action == 'start':
            self._do_position_start(cmd)
        elif mode == 'torque' and action == 'start':
            self._do_torque_start(cmd)
        else:
            self._ack(f'ERROR unknown mode/action: {mode}/{action}')

    # ── Velocity start ────────────────────────────────────
    def _do_velocity_start(self, cmd: dict):
        try:
            acc_l = int(cmd.get('acc_l',    500))
            acc_r = int(cmd.get('acc_r',    500))
            dcc_l = int(cmd.get('dcc_l',    500))
            dcc_r = int(cmd.get('dcc_r',    500))
            tgt_l = int(cmd.get('target_l', 0))
            tgt_r = int(cmd.get('target_r', 0))

            self.driver.clear_fault()
            self.driver.write_velocity(
                left_rpm=tgt_l,
                right_rpm=tgt_r,
                acc_left_ms=acc_l,
                acc_right_ms=acc_r,
                dcc_left_ms=dcc_l,
                dcc_right_ms=dcc_r,
                auto_enable=True,
            )

            self._ui_controlled = True
            self._ui_mode       = 'velocity'
            self.motor_enabled  = True
            self.sent_rpm_l     = tgt_l
            self.sent_rpm_r     = tgt_r
            self.target_rpm_l   = tgt_l
            self.target_rpm_r   = tgt_r

            self._ack('OK')
            self.logger_csv.log('velocity', f'start L={tgt_l} R={tgt_r}', 'OK')
            self.get_logger().info(
                f'UI velocity start: L={tgt_l} R={tgt_r} RPM  '
                f'[acc={acc_l}/{acc_r} ms  dcc={dcc_l}/{dcc_r} ms]'
            )
        except Exception as e:
            self._ack(f'ERROR {e}')
            self.logger_csv.log('velocity', 'start', f'ERROR {e}')

    # ── Position start ────────────────────────────────────
    def _do_position_start(self, cmd: dict):
        try:
            sync     = bool(cmd.get('sync',     True))
            absolute = bool(cmd.get('absolute', False))
            acc_l    = int(cmd.get('acc_l',   500))
            acc_r    = int(cmd.get('acc_r',   500))
            dcc_l    = int(cmd.get('dcc_l',   500))
            dcc_r    = int(cmd.get('dcc_r',   500))
            tgt_l    = int(cmd.get('target_l', 0))
            tgt_r    = int(cmd.get('target_r', 0))
            spd_l    = int(cmd.get('speed_l', 120))
            spd_r    = int(cmd.get('speed_r', 120))

            self.driver.clear_fault()
            self.driver.write_position(
                left_counts=tgt_l,
                right_counts=tgt_r,
                speed_left_rpm=spd_l,
                speed_right_rpm=spd_r,
                acc_left_ms=acc_l,
                acc_right_ms=acc_r,
                dcc_left_ms=dcc_l,
                dcc_right_ms=dcc_r,
                absolute=absolute,
                synchronous=sync,
                auto_enable=True,
                auto_start=True,
            )

            self._ui_controlled = True
            self._ui_mode       = 'position'
            self.motor_enabled  = True

            self._ack('OK')
            mode_str = f'{"absolute" if absolute else "relative"} {"sync" if sync else "async"}'
            self.logger_csv.log('position', f'start {mode_str} L={tgt_l} R={tgt_r}', 'OK')
            self.get_logger().info(f'UI position start: {mode_str} L={tgt_l} R={tgt_r} pulses')
        except Exception as e:
            self._ack(f'ERROR {e}')
            self.logger_csv.log('position', 'start', f'ERROR {e}')

    # ── Torque start ──────────────────────────────────────
    def _do_torque_start(self, cmd: dict):
        try:
            slope_l = int(cmd.get('slope_l', 300))
            slope_r = int(cmd.get('slope_r', 300))
            tgt_l   = int(cmd.get('target_l', 0))
            tgt_r   = int(cmd.get('target_r', 0))

            self.driver.clear_fault()
            self.driver.write_torque(
                left_ma=tgt_l,
                right_ma=tgt_r,
                slope_left_ma_s=slope_l,
                slope_right_ma_s=slope_r,
                auto_enable=True,
            )

            self._ui_controlled = True
            self._ui_mode       = 'torque'
            self.motor_enabled  = True

            self._ack('OK')
            self.logger_csv.log('torque', f'start L={tgt_l} R={tgt_r} mA', 'OK')
            self.get_logger().info(f'UI torque start: L={tgt_l} R={tgt_r} mA')
        except Exception as e:
            self._ack(f'ERROR {e}')
            self.logger_csv.log('torque', 'start', f'ERROR {e}')

    # ── Stop ──────────────────────────────────────────────
    def _do_stop(self):
        try:
            self.driver.set_speed(0, 0)
            self.driver.stop()
            self._ui_controlled = False
            self.motor_enabled  = False
            self.target_rpm_l   = 0
            self.target_rpm_r   = 0
            self.sent_rpm_l     = 0
            self.sent_rpm_r     = 0
            self._ack('OK')
            self.logger_csv.log(self._ui_mode, 'stop', 'OK')
        except Exception as e:
            self._ack(f'ERROR {e}')

    def _do_emergency_stop(self):
        try:
            self.driver.emergency_stop()
            self._ui_controlled = False
            self.motor_enabled  = False
            self.target_rpm_l   = 0
            self.target_rpm_r   = 0
            self.sent_rpm_l     = 0
            self.sent_rpm_r     = 0
            self._ack('OK')
            self.logger_csv.log(self._ui_mode, 'emergency_stop', 'OK')
            self.get_logger().warn('UI: EMERGENCY STOP')
        except Exception as e:
            self._ack(f'ERROR {e}')

    def _do_clear_error(self):
        try:
            self.driver.clear_fault()
            self._ack('OK')
            self.logger_csv.log(self._ui_mode, 'clear_error', 'OK')
        except Exception as e:
            self._ack(f'ERROR {e}')

    def _do_reset_data(self):
        """
        Reset semua state monitoring ke nilai nol.

        Tidak mengirim perintah apapun ke driver ZLAC8015D.
        Hanya membersihkan cache in-memory di node ini.
        Nilai akan kembali diperbarui secara otomatis oleh timer pembaca
        (read_motion_state, read_voltage_timer, read_temp_timer, read_error_code_timer,
         read_status_word_timer, read_torque_timer).
        """
        self.driver.reset_feedback_position(left=True, right=True)
        # Encoder dan odometry TIDAK direset — itu tugas reset_odometry_cb
        self._ack('OK')
        self.logger_csv.log(self._ui_mode, 'reset_data', 'OK')
        self.get_logger().info('Monitoring state reset to zero (encoder/odom NOT affected)')

    def _ack(self, msg: str):
        self.pub_ack.publish(String(data=msg))

    # ═══════════════════════════════════════════════════════
    # NAVIGATION CALLBACKS
    # ═══════════════════════════════════════════════════════

    def enable_srv_cb(self, req, resp):
        if req.data:
            self.driver.clear_fault()
            self.driver.enable()
            self._ui_controlled = False
            self.motor_enabled  = True
            self._ui_mode       = 'velocity'
            resp.message        = 'Motor ENABLED'
            self.get_logger().info(resp.message)
        else:
            self.driver.set_speed(0, 0)
            self.driver.stop()
            self._ui_controlled = False
            self.motor_enabled  = False
            self.sent_rpm_l     = 0
            self.sent_rpm_r     = 0
            resp.message        = 'Motor DISABLED'
            self.get_logger().warn(resp.message)
        resp.success = True
        return resp

    def cmd_cb(self, msg):
        self.cmd              = msg
        self.last_cmd_time    = self.get_clock().now()
        self.cmd_vel_received = True

    def reset_odometry_cb(self, req, resp):
        if self.enc_l is None or self.enc_r is None:
            resp.success = False
            resp.message = 'Encoder not ready'
            return resp
        self.x = self.y = self.yaw = 0.0
        self.prev_enc_l     = self.enc_l
        self.prev_enc_r     = self.enc_r
        self.last_odom_time = self.get_clock().now()
        resp.success = True
        resp.message = 'Odometry reset'
        return resp

    # ═══════════════════════════════════════════════════════
    # CONTROL LOOP (cmd_vel → RPM, 50 Hz)  – NAV STACK ONLY
    # ═══════════════════════════════════════════════════════

    def control_loop(self):
        """
        Hitung target_rpm dari /cmd_vel.
        SKIP jika motor_enabled=False atau _ui_controlled=True.
        """
        if self._shutting_down or not self.motor_enabled:
            self.target_rpm_l = 0
            self.target_rpm_r = 0
            return

        if self._ui_controlled:
            return

        dt       = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        rpm_l, rpm_r = 0, 0

        if self.cmd_vel_received and dt <= self.get_parameter('cmd_vel_timeout').value:
            rpm_l, rpm_r = twist_to_rpm(
                self.cmd.linear.x,
                self.cmd.angular.z,
                self.get_parameter('wheel_radius').value,
                self.get_parameter('wheel_base').value,
            )

        max_rpm           = self.get_parameter('max_rpm').value
        self.target_rpm_l = -int(max(min(rpm_l, max_rpm), -max_rpm))
        self.target_rpm_r  = int(max(min(rpm_r, max_rpm), -max_rpm))

    # ═══════════════════════════════════════════════════════
    # MODBUS WRITE LOOP (20 Hz)  – NAV STACK ONLY
    # ═══════════════════════════════════════════════════════

    def modbus_write_loop(self):
        """
        Kirim target_rpm ke driver.
        SKIP jika motor_enabled=False atau _ui_controlled=True.
        """
        if self._shutting_down or not self.motor_enabled:
            return

        if self._ui_controlled:
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

    # ═══════════════════════════════════════════════════════
    # READ MOTION STATE (20 Hz)
    # ═══════════════════════════════════════════════════════

    def read_motion_state(self):
        if self._shutting_down:
            return
        try:
            vl, vr = self.driver.read_actual_speed()
            if vl is not None:
                self.vel_actual_l = round(vl * 10)
                self.vel_actual_r = round(vr * 10)

            el, er = self.driver.read_encoder()
            if el is not None:
                self.enc_l        = el
                self.enc_r        = er
                self.pos_actual_l = el
                self.pos_actual_r = er
        except KeyboardInterrupt:
            return
        except Exception:
            pass

    # ═══════════════════════════════════════════════════════
    # MONITORING TIMERS
    # ═══════════════════════════════════════════════════════

    def read_voltage_timer(self):
        if self._shutting_down:
            return
        try:
            v = self.driver.read_voltage()
            if v is not None:
                # simpan raw: 0.01V per unit
                self.bus_voltage_raw = int(round(v / 0.01))
        except KeyboardInterrupt:
            return
        except Exception:
            pass

    def read_error_code_timer(self):
        if self._shutting_down:
            return
        try:
            el, er = self.driver.read_error_code()
            self.error_code_l = el
            self.error_code_r = er
        except KeyboardInterrupt:
            return
        except Exception:
            pass

    def read_temp_timer(self):
        if self._shutting_down:
            return
        try:
            tl, tr = self.driver.read_motor_temperature()
            if tl is not None:
                self.motor_temp_l = tl
                self.motor_temp_r = tr

            dt = self.driver.read_driver_temperature()
            if dt is not None:
                # simpan raw: 0.1°C per unit
                self.driver_temp_raw = int(round(dt / 0.1))
        except KeyboardInterrupt:
            return
        except Exception:
            pass

    def read_status_word_timer(self):
        """
        Baca status word register 0x20A2 dari driver.
        Bits [7:6] = drive state:
          0x00 = Shaft Release
          0x40 = Shaft Lock (enabled)
          0x80 = Emergency Stop
          0xC0 = Alarm
        (NEW v4)
        """
        if self._shutting_down:
            return
        try:
            sw = self.driver.read_status_word()
            if sw is not None:
                self.status_word = sw
        except KeyboardInterrupt:
            return
        except Exception:
            pass

    def read_torque_timer(self):
        """
        Baca actual torque Left/Right dari register 0x20AD dan 0x20AE.
        Nilai dalam 0.1 A.
        (NEW v4)
        """
        if self._shutting_down:
            return
        try:
            tl, tr = self.driver.read_actual_torque()
            if tl is not None:
                self.torque_actual_l = tl
                self.torque_actual_r = tr
        except KeyboardInterrupt:
            return
        except Exception:
            pass

    # ═══════════════════════════════════════════════════════
    # PUBLISH bldc/status (2 Hz)
    # ═══════════════════════════════════════════════════════

    def publish_bldc_status(self):
        if self._shutting_down:
            return
        msg = Float32MultiArray()
        msg.data = [
            float(self.vel_actual_l),      # [0]  0.1 r/min
            float(self.vel_actual_r),      # [1]  0.1 r/min
            float(self.pos_actual_l),      # [2]  counts
            float(self.pos_actual_r),      # [3]  counts
            float(self.torque_actual_l),   # [4]  0.1 A raw
            float(self.torque_actual_r),   # [5]  0.1 A raw
            float(self.motor_temp_l),      # [6]  °C
            float(self.motor_temp_r),      # [7]  °C
            float(self.driver_temp_raw),   # [8]  0.1 °C raw
            float(self.bus_voltage_raw),   # [9]  0.01 V raw
            float(self.status_word),       # [10] 0x20A2
            float(self.error_code_l),      # [11] 0x20A5
            float(self.error_code_r),      # [12] 0x20A6
        ]
        self.pub_status.publish(msg)

    # ═══════════════════════════════════════════════════════
    # PUBLISH bldc/connection (1 Hz)
    # ═══════════════════════════════════════════════════════

    def publish_connection(self):
        if self._shutting_down:
            return
        try:
            v = self.driver.read_voltage()
            if v is not None:
                self._driver_ok = True
                self.pub_conn.publish(String(data='OK'))
            else:
                self._driver_ok = False
                self.pub_conn.publish(String(data='DISCONNECTED'))
        except Exception:
            self._driver_ok = False
            self.pub_conn.publish(String(data='FAULT'))

    # ═══════════════════════════════════════════════════════
    # ODOMETRY
    # ═══════════════════════════════════════════════════════

    def update_odometry(self):
        if self._shutting_down or self.enc_l is None or self.enc_r is None:
            return

        now = self.get_clock().now()

        if self.prev_enc_l is None:
            self.prev_enc_l     = self.enc_l
            self.prev_enc_r     = self.enc_r
            self.last_odom_time = now
            return

        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        dl = -(self.enc_l - self.prev_enc_l)
        dr =  (self.enc_r - self.prev_enc_r)
        self.prev_enc_l     = self.enc_l
        self.prev_enc_r     = self.enc_r
        self.last_odom_time = now

        r   = self.get_parameter('wheel_radius').value
        b   = self.get_parameter('wheel_base').value
        cpr = self.get_parameter('encoder_cpr').value

        ds_l   = r * (2.0 * math.pi * dl / cpr)
        ds_r   = r * (2.0 * math.pi * dr / cpr)
        ds     = 0.5 * (ds_l + ds_r)
        dtheta = (ds_r - ds_l) / b

        self.x   += ds * math.cos(self.yaw + 0.5 * dtheta)
        self.y   += ds * math.sin(self.yaw + 0.5 * dtheta)
        self.yaw += dtheta
        self.yaw  = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.publish_odom(now, ds / dt, dtheta / dt)

    def publish_odom(self, stamp_time, v, w):
        stamp = stamp_time.to_msg()

        q = Quaternion()
        q.z = math.sin(self.yaw * 0.5)
        q.w = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp          = stamp
        odom.header.frame_id       = 'odom'
        odom.child_frame_id        = 'base_link'
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp            = stamp
        t.header.frame_id         = 'odom'
        t.child_frame_id          = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation      = q
        self.tf_broadcaster.sendTransform(t)

    # ═══════════════════════════════════════════════════════
    # DIAGNOSTICS
    # ═══════════════════════════════════════════════════════

    def diagnostic_cb(self, stat: DiagnosticStatusWrapper):
        src = 'UI' if self._ui_controlled else 'nav'
        if self._driver_ok and self.motor_enabled:
            stat.summary(stat.OK, f'Motor ENABLED – mode: {self._ui_mode} [{src}]')
        elif self._driver_ok:
            stat.summary(stat.WARN, 'Motor DISABLED – driver OK')
        else:
            stat.summary(stat.ERROR, 'Driver FAULT / DISCONNECTED')
        stat.add('mode',          self._ui_mode)
        stat.add('ui_controlled', str(self._ui_controlled))
        stat.add('vel_l',         str(self.vel_actual_l))
        stat.add('vel_r',         str(self.vel_actual_r))
        stat.add('torque_l',      str(self.torque_actual_l))
        stat.add('torque_r',      str(self.torque_actual_r))
        stat.add('status_word',   hex(self.status_word))
        stat.add('error_l',       hex(self.error_code_l))
        stat.add('error_r',       hex(self.error_code_r))
        return stat


# ═══════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = AMRMotorNode()

    def shutdown_hook():
        node.get_logger().warn('ROS shutdown → stop motor')
        node._shutting_down = True
        try:
            node.driver.set_speed(0, 0)
            node.driver.stop()
            node.driver.close()
        except Exception:
            pass

    rclpy.get_default_context().on_shutdown(shutdown_hook)

    try:
        rclpy.spin(node)
    finally:
        node.logger_csv.close()
        node.destroy_node()
