#!/usr/bin/env python3
"""
zlac8015d.py  –  ZLAC8015D Modbus RTU Driver
=============================================

Disesuaikan dengan referensi zlac_monitor.py:
  - Gunakan pymodbus langsung (ModbusSerialClient), bukan wrapper custom
  - Read block sekaligus (FC03 batch) untuk efisiensi
  - Konversi nilai signed benar (u16_to_i16)
  - Driver temperature → signed I16 × 0.1 °C
  - Motor temperature → high byte = Left, low byte = Right (signed I8 per byte)
  - Error code kiri/kanan dari register terpisah (0x20A5 / 0x20A6)
  - write_register  → FC06 (write single)
  - write_registers → FC16 (write multiple) untuk block write

Dependency:
  pip install pymodbus==3.5.2
"""

from __future__ import annotations

import logging
import time
from enum import IntEnum
from typing import Optional, Tuple

from pymodbus.client import ModbusSerialClient


# ─────────────────────────────────────────────────────────────
# REGISTER MAP  (datasheet address, langsung / direct)
# ─────────────────────────────────────────────────────────────

class Reg(IntEnum):
    # Control
    CLEAR_FEEDBACK_POSITION   = 0x2005
    SHAFT_STATE_AFTER_POWER_ON = 0x2007
    CONTROL_MODE              = 0x200D
    CONTROL_WORD              = 0x200E
    SYNC_ASYNC                = 0x200F
    IO_ESTOP_MODE             = 0x2021

    # Accel / decel
    ACC_LEFT                  = 0x2080
    ACC_RIGHT                 = 0x2081
    DCC_LEFT                  = 0x2082
    DCC_RIGHT                 = 0x2083
    QUICK_STOP_LEFT           = 0x2084
    QUICK_STOP_RIGHT          = 0x2085

    # Torque slope
    TORQUE_SLOPE_LEFT         = 0x2086
    TORQUE_SLOPE_RIGHT        = 0x2087

    # Target velocity
    TARGET_VEL_L              = 0x2088
    TARGET_VEL_R              = 0x2089

    # Target position (32-bit split)
    TARGET_POS_HI_L           = 0x208A
    TARGET_POS_LO_L           = 0x208B
    TARGET_POS_HI_R           = 0x208C
    TARGET_POS_LO_R           = 0x208D

    # Max speed in position mode
    TARGET_SPEED_L            = 0x208E
    TARGET_SPEED_R            = 0x208F

    # Target torque
    TARGET_TORQUE_L           = 0x2090
    TARGET_TORQUE_R           = 0x2091

    # Read-only
    BUS_VOLTAGE               = 0x20A1
    STATUS_WORD               = 0x20A2
    MOTOR_TEMPERATURE         = 0x20A4   # High byte = Left, Low byte = Right
    ERROR_CODE_L              = 0x20A5
    ERROR_CODE_R              = 0x20A6

    ACTUAL_POS_HI_L           = 0x20A7
    ACTUAL_POS_LO_L           = 0x20A8
    ACTUAL_POS_HI_R           = 0x20A9
    ACTUAL_POS_LO_R           = 0x20AA

    ACTUAL_VEL_L              = 0x20AB   # unit 0.1 r/min, I16
    ACTUAL_VEL_R              = 0x20AC

    ACTUAL_TORQUE_L           = 0x20AD   # unit 0.1 A, I16
    ACTUAL_TORQUE_R           = 0x20AE

    DRIVER_TEMPERATURE        = 0x20B0   # unit 0.1 °C, I16


# Read-block: baca sekaligus dari BUS_VOLTAGE s.d. DRIVER_TEMPERATURE
_READ_BLOCK_START = Reg.BUS_VOLTAGE
_READ_BLOCK_END   = Reg.DRIVER_TEMPERATURE
_READ_BLOCK_COUNT = int(_READ_BLOCK_END) - int(_READ_BLOCK_START) + 1


# ─────────────────────────────────────────────────────────────
# CONTROL WORD VALUES
# ─────────────────────────────────────────────────────────────

class ControlWord(IntEnum):
    EMERGENCY_STOP = 0x05
    CLEAR_FAULT    = 0x06
    STOP           = 0x07
    ENABLE         = 0x08
    START_SYNC     = 0x10   # position mode synchronous
    START_LEFT     = 0x11
    START_RIGHT    = 0x12


# ─────────────────────────────────────────────────────────────
# CONTROL MODE VALUES
# ─────────────────────────────────────────────────────────────

class ZLACMode(IntEnum):
    POSITION_RELATIVE = 1
    POSITION_ABSOLUTE = 2
    VELOCITY          = 3
    TORQUE            = 4


# ─────────────────────────────────────────────────────────────
# ERROR CODE DECODE
# ─────────────────────────────────────────────────────────────

ERROR_BITS = {
    0x0001: "Over Voltage",
    0x0002: "Under Voltage",
    0x0004: "Over Current",
    0x0008: "Over Load",
    0x0010: "Current Out of Tolerance",
    0x0020: "Encoder Out of Tolerance",
    0x0040: "Velocity Out of Tolerance",
    0x0080: "Reference Voltage Error",
    0x0100: "EEPROM Error",
    0x0200: "Hall Error",
    0x0400: "Motor Over Temperature",
}

_KNOWN_ERROR_MASK = 0
for _b in ERROR_BITS:
    _KNOWN_ERROR_MASK |= _b


def decode_error(code: int) -> str:
    if code == 0:
        return "OK"
    msgs = [txt for bit, txt in ERROR_BITS.items() if code & bit]
    unk = code & ~_KNOWN_ERROR_MASK
    if unk:
        msgs.append(f"UnknownBits(0x{unk:04X})")
    return " | ".join(msgs) if msgs else f"Unknown(0x{code:04X})"


# ─────────────────────────────────────────────────────────────
# STATUS WORD DECODE
# ─────────────────────────────────────────────────────────────

_DRIVE_STATE_MAP = {
    0x00: "Shaft Release",
    0x40: "Shaft Lock",
    0x80: "Emergency Stop",
    0xC0: "Alarm",
}


def decode_status_word(status_word: int) -> tuple[str, str]:
    """
    Status Word 0x20A2:
      Left  state  : bit7, bit6  (low byte)
      Right state  : bit15, bit14 (high byte)
      Left  running: bit0
      Right running: bit8
    """
    left_byte  = status_word & 0x00FF
    right_byte = (status_word >> 8) & 0x00FF

    left_state  = _DRIVE_STATE_MAP.get(left_byte  & 0xC0, f"Unknown(0x{left_byte  & 0xC0:02X})")
    right_state = _DRIVE_STATE_MAP.get(right_byte & 0xC0, f"Unknown(0x{right_byte & 0xC0:02X})")

    left_run  = "Running" if (status_word & 0x0001) else "Stopped"
    right_run = "Running" if (status_word & 0x0100) else "Stopped"

    return f"{left_state} / {left_run}", f"{right_state} / {right_run}"


# ─────────────────────────────────────────────────────────────
# DATA TYPE HELPERS
# ─────────────────────────────────────────────────────────────

def _u16_to_i16(v: int) -> int:
    """Unsigned 16-bit register → signed int16."""
    return v if v < 0x8000 else v - 0x10000


def _i16_to_u16(v: int) -> int:
    """Signed int16 → unsigned 16-bit (untuk dikirim ke register)."""
    if not -32768 <= v <= 32767:
        raise ValueError(f"I16 out of range: {v}")
    return v & 0xFFFF


def _u8_to_i8(v: int) -> int:
    """Unsigned 8-bit → signed int8 (untuk motor temperature per byte)."""
    return v if v < 128 else v - 256


def _i32_to_u16_pair(v: int) -> tuple[int, int]:
    """Signed int32 → (high_u16, low_u16)."""
    if not -0x80000000 <= v <= 0x7FFFFFFF:
        raise ValueError(f"I32 out of range: {v}")
    raw = v & 0xFFFFFFFF
    return (raw >> 16) & 0xFFFF, raw & 0xFFFF


def _combine_i32(hi: int, lo: int) -> int:
    """Dua register U16 (hi, lo) → signed int32."""
    raw = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)
    return raw if raw < 0x80000000 else raw - 0x100000000


def _clamp_rpm(rpm: int, limit: int = 3000) -> int:
    return max(-limit, min(limit, int(rpm)))


# ─────────────────────────────────────────────────────────────
# EXCEPTION
# ─────────────────────────────────────────────────────────────

class ZLACCommunicationError(RuntimeError):
    pass


# ─────────────────────────────────────────────────────────────
# DRIVER CLASS
# ─────────────────────────────────────────────────────────────

class ZLAC8015D:
    """
    ZLAC8015D Modbus RTU Driver – ROS-compatible

    Menggunakan pymodbus.ModbusSerialClient secara langsung
    (tidak melalui wrapper custom), sama dengan zlac_monitor.py.

    Semua pembacaan nilai signed (velocity, torque, temperature)
    menggunakan u16_to_i16() sesuai datasheet.

    CATATAN PENTING:
      - Target velocity BOLEH dikirim via FC06 (write_register) satu-satu,
        MAUPUN via FC16 (write_registers) untuk dua motor sekaligus.
      - Untuk blok ACC/DCC (4 register berurutan), selalu gunakan FC16.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        slave_id: int = 1,
        timeout_s: float = 1.0,
        retries: int = 3,
        retry_delay_s: float = 0.05,
    ) -> None:
        self.port          = port
        self.baudrate      = baudrate
        self.slave_id      = slave_id
        self.timeout_s     = timeout_s
        self.retries       = retries
        self.retry_delay_s = retry_delay_s

        self.client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout_s,
        )

    # ── Context manager ───────────────────────────────────────

    def __enter__(self) -> "ZLAC8015D":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def connect(self) -> None:
        if not self.client.connect():
            raise ZLACCommunicationError(f"Cannot open serial port {self.port}")
        logging.info(
            "ZLAC8015D connected: port=%s baudrate=%s slave=%s",
            self.port, self.baudrate, self.slave_id,
        )

    def close(self) -> None:
        self.client.close()
        logging.info("ZLAC8015D serial port closed")

    # ── Low-level Modbus primitives ───────────────────────────

    def read_holding_registers(self, ds_addr: int | Reg, count: int) -> list[int]:
        """FC03 – baca `count` register mulai dari `ds_addr` (datasheet address)."""
        last_error = None
        for attempt in range(1, self.retries + 1):
            result = self.client.read_holding_registers(
                address=int(ds_addr),
                count=count,
                slave=self.slave_id,
            )
            if result is not None and not result.isError():
                return list(result.registers)

            last_error = result
            logging.warning(
                "FC03 read failed attempt %d/%d addr=0x%04X count=%d err=%s",
                attempt, self.retries, int(ds_addr), count, result,
            )
            time.sleep(self.retry_delay_s)

        raise ZLACCommunicationError(
            f"FC03 read failed addr=0x{int(ds_addr):04X} count={count} last={last_error}"
        )

    def write_register(self, ds_addr: int | Reg, value: int) -> None:
        """FC06 – tulis satu register 16-bit."""
        last_error = None
        val16 = value & 0xFFFF
        for attempt in range(1, self.retries + 1):
            result = self.client.write_register(
                address=int(ds_addr),
                value=val16,
                slave=self.slave_id,
            )
            if result is not None and not result.isError():
                return

            last_error = result
            logging.warning(
                "FC06 write failed attempt %d/%d addr=0x%04X value=0x%04X err=%s",
                attempt, self.retries, int(ds_addr), val16, result,
            )
            time.sleep(self.retry_delay_s)

        raise ZLACCommunicationError(
            f"FC06 write failed addr=0x{int(ds_addr):04X} value=0x{val16:04X} last={last_error}"
        )

    def write_registers(self, ds_addr: int | Reg, values: list[int]) -> None:
        """FC16 – tulis beberapa register sekaligus (block write)."""
        last_error = None
        payload = [v & 0xFFFF for v in values]
        for attempt in range(1, self.retries + 1):
            result = self.client.write_registers(
                address=int(ds_addr),
                values=payload,
                slave=self.slave_id,
            )
            if result is not None and not result.isError():
                return

            last_error = result
            logging.warning(
                "FC16 write failed attempt %d/%d addr=0x%04X values=%s err=%s",
                attempt, self.retries, int(ds_addr),
                [f"0x{x:04X}" for x in payload], result,
            )
            time.sleep(self.retry_delay_s)

        raise ZLACCommunicationError(
            f"FC16 write failed addr=0x{int(ds_addr):04X} values={payload} last={last_error}"
        )

    # ── Read-all (batch, efisien) ─────────────────────────────

    def read_all(self) -> dict:
        """
        Baca semua parameter read-only dalam satu FC03 request.
        Return dict berisi semua nilai yang sudah dikonversi.
        Cocok untuk monitoring loop.
        """
        regs = self.read_holding_registers(_READ_BLOCK_START, _READ_BLOCK_COUNT)

        def at(reg: Reg) -> int:
            return regs[int(reg) - int(_READ_BLOCK_START)]

        bus_voltage_v       = at(Reg.BUS_VOLTAGE) * 0.01

        status_word         = at(Reg.STATUS_WORD)
        left_status, right_status = decode_status_word(status_word)

        motor_temp_raw      = at(Reg.MOTOR_TEMPERATURE)
        motor_temp_l        = float(_u8_to_i8((motor_temp_raw >> 8) & 0xFF))
        motor_temp_r        = float(_u8_to_i8(motor_temp_raw & 0xFF))

        error_l             = at(Reg.ERROR_CODE_L)
        error_r             = at(Reg.ERROR_CODE_R)

        pos_l = _combine_i32(at(Reg.ACTUAL_POS_HI_L), at(Reg.ACTUAL_POS_LO_L))
        pos_r = _combine_i32(at(Reg.ACTUAL_POS_HI_R), at(Reg.ACTUAL_POS_LO_R))

        vel_l_rpm           = _u16_to_i16(at(Reg.ACTUAL_VEL_L)) * 0.1
        vel_r_rpm           = _u16_to_i16(at(Reg.ACTUAL_VEL_R)) * 0.1

        torque_l_a          = _u16_to_i16(at(Reg.ACTUAL_TORQUE_L)) * 0.1
        torque_r_a          = _u16_to_i16(at(Reg.ACTUAL_TORQUE_R)) * 0.1

        driver_temp_c       = _u16_to_i16(at(Reg.DRIVER_TEMPERATURE)) * 0.1

        return {
            "bus_voltage_v":      bus_voltage_v,
            "status_word":        status_word,
            "left_status":        left_status,
            "right_status":       right_status,
            "motor_temp_l_c":     motor_temp_l,
            "motor_temp_r_c":     motor_temp_r,
            "error_l":            error_l,
            "error_r":            error_r,
            "error_l_text":       decode_error(error_l),
            "error_r_text":       decode_error(error_r),
            "pos_l_counts":       pos_l,
            "pos_r_counts":       pos_r,
            "vel_l_rpm":          vel_l_rpm,
            "vel_r_rpm":          vel_r_rpm,
            "torque_l_a":         torque_l_a,
            "torque_r_a":         torque_r_a,
            "driver_temp_c":      driver_temp_c,
        }

    # ── Individual reads (untuk backward-compat dengan node lama) ──

    def read_actual_speed(self) -> tuple[Optional[float], Optional[float]]:
        """
        Actual velocity Left & Right dalam RPM.
        Register 0x20AB–0x20AC, unit 0.1 r/min, I16.
        """
        try:
            regs = self.read_holding_registers(Reg.ACTUAL_VEL_L, 2)
            vel_l = _u16_to_i16(regs[0]) * 0.1
            vel_r = _u16_to_i16(regs[1]) * 0.1
            return vel_l, vel_r
        except ZLACCommunicationError:
            return None, None

    def read_encoder(self) -> tuple[Optional[int], Optional[int]]:
        """
        Encoder position Left & Right (signed int32, counts).
        Register 0x20A7–0x20AA (4 register berurutan).
        """
        try:
            regs  = self.read_holding_registers(Reg.ACTUAL_POS_HI_L, 4)
            pos_l = _combine_i32(regs[0], regs[1])
            pos_r = _combine_i32(regs[2], regs[3])
            return pos_l, pos_r
        except ZLACCommunicationError:
            return None, None

    def read_voltage(self) -> Optional[float]:
        """
        Bus voltage (V).
        Register 0x20A1, unit 0.01 V, U16.
        """
        try:
            regs = self.read_holding_registers(Reg.BUS_VOLTAGE, 1)
            return regs[0] * 0.01
        except ZLACCommunicationError:
            return None

    def read_driver_temperature(self) -> Optional[float]:
        """
        Driver temperature (°C).
        Register 0x20B0, unit 0.1 °C, I16 (signed).
        """
        try:
            regs = self.read_holding_registers(Reg.DRIVER_TEMPERATURE, 1)
            return _u16_to_i16(regs[0]) * 0.1
        except ZLACCommunicationError:
            return None

    def read_motor_temperature(self) -> tuple[Optional[float], Optional[float]]:
        """
        Motor temperature Left & Right (°C).
        Register 0x20A4:
          High byte = Left motor temperature (signed I8)
          Low byte  = Right motor temperature (signed I8)
        """
        try:
            regs  = self.read_holding_registers(Reg.MOTOR_TEMPERATURE, 1)
            val   = regs[0]
            temp_l = float(_u8_to_i8((val >> 8) & 0xFF))
            temp_r = float(_u8_to_i8(val & 0xFF))
            return temp_l, temp_r
        except ZLACCommunicationError:
            return None, None

    def read_error_code(self) -> tuple[int, int]:
        """
        Error code Left & Right.
        Register 0x20A5 (Left), 0x20A6 (Right).
        Baca 2 register sekaligus (lebih efisien dari 2 × FC03).
        """
        try:
            regs   = self.read_holding_registers(Reg.ERROR_CODE_L, 2)
            err_l  = regs[0]
            err_r  = regs[1]
            return err_l, err_r
        except ZLACCommunicationError as e:
            raise RuntimeError(f"Failed to read error code: {e}") from e

    def read_status_word(self) -> int:
        """Status word 0x20A2 (U16)."""
        regs = self.read_holding_registers(Reg.STATUS_WORD, 1)
        return regs[0]

    def read_actual_torque(self) -> tuple[Optional[float], Optional[float]]:
        """
        Actual torque Left & Right (A).
        Register 0x20AD–0x20AE, unit 0.1 A, I16.
        """
        try:
            regs     = self.read_holding_registers(Reg.ACTUAL_TORQUE_L, 2)
            torque_l = _u16_to_i16(regs[0]) * 0.1
            torque_r = _u16_to_i16(regs[1]) * 0.1
            return torque_l, torque_r
        except ZLACCommunicationError:
            return None, None

    # ── Control words ─────────────────────────────────────────

    def enable(self) -> None:
        self.write_register(Reg.CONTROL_WORD, ControlWord.ENABLE)

    def stop(self) -> None:
        self.write_register(Reg.CONTROL_WORD, ControlWord.STOP)

    def emergency_stop(self) -> None:
        self.write_register(Reg.CONTROL_WORD, ControlWord.EMERGENCY_STOP)

    def clear_fault(self) -> None:
        self.write_register(Reg.CONTROL_WORD, ControlWord.CLEAR_FAULT)

    def free_shaft(self) -> None:
        """Release shaft sehingga bebas berputar."""
        self.write_register(Reg.SHAFT_STATE_AFTER_POWER_ON, 0)
        self.write_register(Reg.IO_ESTOP_MODE, 1)
        self.stop()

    def reset_feedback_position(self, left: bool = True, right: bool = True) -> None:
        """Clear feedback position (0x2005)."""
        if left and right:
            value = 3
        elif left:
            value = 1
        elif right:
            value = 2
        else:
            raise ValueError("Select at least left or right")
        self.write_register(Reg.CLEAR_FEEDBACK_POSITION, value)

    # ── Mode setup ────────────────────────────────────────────

    def set_velocity_mode(self) -> None:
        self.write_register(Reg.CONTROL_MODE, ZLACMode.VELOCITY)

    def set_position_mode(self, absolute: bool = False, synchronous: bool = False) -> None:
        sync_val = 0 if synchronous else 1   # 0=sync, 1=async
        mode     = ZLACMode.POSITION_ABSOLUTE if absolute else ZLACMode.POSITION_RELATIVE
        self.write_register(Reg.SYNC_ASYNC, sync_val)
        self.write_register(Reg.CONTROL_MODE, mode)

    def set_torque_mode(self) -> None:
        self.write_register(Reg.CONTROL_MODE, ZLACMode.TORQUE)

    # ── Accel / decel profile ─────────────────────────────────

    def set_accel_profile(
        self,
        accel_ms: int    = 1000,
        decel_ms: int    = 800,
        quick_stop_ms: int = 10,
    ) -> None:
        """
        Set acceleration, deceleration, dan quick-stop time (ms).
        Menggunakan FC16 (write_registers) untuk efisiensi:
          Tulis ACC_LEFT (0x2080) s.d. DCC_RIGHT (0x2083) → 4 register sekaligus.
          Tulis QUICK_STOP_LEFT (0x2084) dan QUICK_STOP_RIGHT (0x2085) → 2 register.
        """
        # 4 register berurutan: acc_l, acc_r, dcc_l, dcc_r
        self.write_registers(Reg.ACC_LEFT, [accel_ms, accel_ms, decel_ms, decel_ms])
        # 2 register berurutan: quick_stop_l, quick_stop_r
        self.write_registers(Reg.QUICK_STOP_LEFT, [quick_stop_ms, quick_stop_ms])

    # ── Velocity command ──────────────────────────────────────

    def set_speed(self, left_rpm: int, right_rpm: int) -> bool:
        """
        Set target velocity (RPM) untuk kedua motor sekaligus via FC16.
        Range: -3000 s.d. 3000 RPM.
        """
        try:
            l = _i16_to_u16(_clamp_rpm(left_rpm))
            r = _i16_to_u16(_clamp_rpm(right_rpm))
            self.write_registers(Reg.TARGET_VEL_L, [l, r])
            return True
        except (ZLACCommunicationError, ValueError):
            return False

    def set_speed_left(self, rpm: int) -> None:
        """Set target velocity motor kiri saja (FC06)."""
        self.write_register(Reg.TARGET_VEL_L, _i16_to_u16(_clamp_rpm(rpm)))

    def set_speed_right(self, rpm: int) -> None:
        """Set target velocity motor kanan saja (FC06)."""
        self.write_register(Reg.TARGET_VEL_R, _i16_to_u16(_clamp_rpm(rpm)))

    # ── Velocity mode – full sequence ─────────────────────────

    def write_velocity(
        self,
        left_rpm: Optional[int],
        right_rpm: Optional[int],
        acc_left_ms: int  = 500,
        acc_right_ms: int = 500,
        dcc_left_ms: int  = 500,
        dcc_right_ms: int = 500,
        auto_enable: bool = True,
    ) -> None:
        """
        Set velocity mode + accel profile + enable + target velocity.
        Sesuai urutan inisialisasi datasheet section 3.1.
        """
        self.set_velocity_mode()
        self.write_registers(Reg.ACC_LEFT, [acc_left_ms, acc_right_ms, dcc_left_ms, dcc_right_ms])

        if auto_enable:
            self.enable()

        if left_rpm is not None and right_rpm is not None:
            self.write_registers(
                Reg.TARGET_VEL_L,
                [_i16_to_u16(_clamp_rpm(left_rpm)), _i16_to_u16(_clamp_rpm(right_rpm))],
            )
        elif left_rpm is not None:
            self.write_register(Reg.TARGET_VEL_L, _i16_to_u16(_clamp_rpm(left_rpm)))
        elif right_rpm is not None:
            self.write_register(Reg.TARGET_VEL_R, _i16_to_u16(_clamp_rpm(right_rpm)))
        else:
            raise ValueError("Minimal salah satu left_rpm atau right_rpm harus diisi")

    # ── Position mode – full sequence ─────────────────────────

    def write_position(
        self,
        left_counts: Optional[int],
        right_counts: Optional[int],
        speed_left_rpm: int  = 120,
        speed_right_rpm: int = 120,
        acc_left_ms: int  = 500,
        acc_right_ms: int = 500,
        dcc_left_ms: int  = 500,
        dcc_right_ms: int = 500,
        absolute: bool    = False,
        synchronous: bool = False,
        auto_enable: bool = True,
        auto_start: bool  = True,
    ) -> None:
        """
        Set position mode + accel profile + target speed + enable + target position + start.
        """
        self.set_position_mode(absolute=absolute, synchronous=synchronous)
        self.write_registers(Reg.ACC_LEFT, [acc_left_ms, acc_right_ms, dcc_left_ms, dcc_right_ms])
        self.write_registers(Reg.TARGET_SPEED_L, [speed_left_rpm, speed_right_rpm])

        if auto_enable:
            self.enable()

        if left_counts is not None and right_counts is not None:
            lhi, llo = _i32_to_u16_pair(left_counts)
            rhi, rlo = _i32_to_u16_pair(right_counts)
            self.write_registers(Reg.TARGET_POS_HI_L, [lhi, llo, rhi, rlo])
            if auto_start:
                if synchronous:
                    self.write_register(Reg.CONTROL_WORD, ControlWord.START_SYNC)
                else:
                    self.write_register(Reg.CONTROL_WORD, ControlWord.START_LEFT)
                    self.write_register(Reg.CONTROL_WORD, ControlWord.START_RIGHT)

        elif left_counts is not None:
            lhi, llo = _i32_to_u16_pair(left_counts)
            self.write_registers(Reg.TARGET_POS_HI_L, [lhi, llo])
            if auto_start:
                self.write_register(Reg.CONTROL_WORD, ControlWord.START_LEFT)

        elif right_counts is not None:
            rhi, rlo = _i32_to_u16_pair(right_counts)
            self.write_registers(Reg.TARGET_POS_HI_R, [rhi, rlo])
            if auto_start:
                self.write_register(Reg.CONTROL_WORD, ControlWord.START_RIGHT)

        else:
            raise ValueError("Minimal salah satu left_counts atau right_counts harus diisi")

    # ── Torque mode – full sequence ───────────────────────────

    def write_torque(
        self,
        left_ma: Optional[int],
        right_ma: Optional[int],
        slope_left_ma_s: int  = 500,
        slope_right_ma_s: int = 500,
        auto_enable: bool = True,
    ) -> None:
        """
        Set torque mode + slope + enable + target torque.
        Unit: mA. Range: -30000 s.d. 30000.
        """
        self.set_torque_mode()
        self.write_registers(Reg.TORQUE_SLOPE_LEFT, [slope_left_ma_s, slope_right_ma_s])

        if auto_enable:
            self.enable()

        if left_ma is not None and right_ma is not None:
            self.write_registers(
                Reg.TARGET_TORQUE_L,
                [_i16_to_u16(left_ma), _i16_to_u16(right_ma)],
            )
        elif left_ma is not None:
            self.write_register(Reg.TARGET_TORQUE_L, _i16_to_u16(left_ma))
        elif right_ma is not None:
            self.write_register(Reg.TARGET_TORQUE_R, _i16_to_u16(right_ma))
        else:
            raise ValueError("Minimal salah satu left_ma atau right_ma harus diisi")