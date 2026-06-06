class ZLAC8015D:
    """
    ZLAC8015D Modbus RTU Driver (ROS-compatible, STABLE version)

    IMPORTANT:
    - Target velocity MUST be written using FC06 (write_single_register)
    - FC16 (write_multiple_registers) is NOT reliable for ZLAC8015D
    """

    # ---------------- Registers ----------------
    REG_CONTROL_MODE     = 0x200D
    REG_CONTROL_WORD     = 0x200E

    REG_TARGET_VEL_L     = 0x2088
    REG_TARGET_VEL_R     = 0x2089

    REG_ACTUAL_VEL_L     = 0x20AB
    REG_ACTUAL_VEL_R     = 0x20AC

    REG_POS_L_HI         = 0x20A7
    REG_POS_L_LO         = 0x20A8
    REG_POS_R_HI         = 0x20A9
    REG_POS_R_LO         = 0x20AA

    REG_BUS_VOLT         = 0x20A1
    REG_DRIVER_TEMP      = 0x20B0
    REG_FAULT_CODE       = 0x20A5

    REG_MOTOR_TEMP       = 0x20A4

    REG_ERROR_CODE_L     = 0x20A5
    REG_ERROR_CODE_R     = 0x20A6


    # -------------------------------------------------

    def __init__(self, modbus, slave_id=1):
        self.mb = modbus
        self.id = slave_id

    # =====================================================
    # MODE & CONTROL
    # =====================================================
    def set_velocity_mode(self):
        # Profile velocity mode
        return self.mb.write_single_register(self.id, self.REG_CONTROL_MODE, 3)

    def enable(self):
        # ENABLE drive (reference script uses 0x0008)
        return self.mb.write_single_register(self.id, self.REG_CONTROL_WORD, 0x08)

    def stop(self):
        # Quick stop / disable torque
        return self.mb.write_single_register(self.id, self.REG_CONTROL_WORD, 0x07)

    def clear_fault(self):
        # Same control word used to clear fault on ZLAC
        return self.mb.write_single_register(self.id, self.REG_CONTROL_WORD, 0x06)

    # =====================================================
    # VELOCITY COMMAND (CRITICAL PART)
    # =====================================================
    def set_speed(self, left_rpm, right_rpm):
        """
        Set target velocity (RPM)
        MUST use FC06 (write_single_register) for each motor.
        """

        def to_u16(val: int) -> int:
            # clamp to int16
            if val > 32767:
                val = 32767
            elif val < -32768:
                val = -32768

            # convert to unsigned 16-bit
            if val < 0:
                val += 0x10000
            return int(val)

        try:
            l = to_u16(int(left_rpm))
            r = to_u16(int(right_rpm))

            self.mb.write_single_register(self.id, self.REG_TARGET_VEL_L, l)
            self.mb.write_single_register(self.id, self.REG_TARGET_VEL_R, r)
            return True
        except Exception:
            return False

    # =====================================================
    # FEEDBACK
    # =====================================================
    def read_actual_speed(self):
        """
        Actual velocity (RPM)
        Unit: 0.1 RPM
        Signed int16
        """
        r = self.mb.read_holding_registers(self.id, self.REG_ACTUAL_VEL_L, 2)
        if not r:
            return None, None

        def i16(v):
            return v - 0x10000 if v > 0x7FFF else v

        left = i16(r[0]) / 10.0
        right = i16(r[1]) / 10.0
        return left, right

    def read_encoder(self):
        """
        Encoder position (SIGNED int32)
        """
        r = self.mb.read_holding_registers(self.id, self.REG_POS_L_HI, 4)
        if not r:
            return None, None

        def i32(hi, lo):
            val = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)
            if val & 0x80000000:
                val -= 0x100000000
            return val

        enc_l = i32(r[0], r[1])
        enc_r = i32(r[2], r[3])
        return enc_l, enc_r

    # =====================================================
    # VOLTAGE & TEMPERATURE
    # =====================================================
    def read_voltage(self):
        """
        Bus voltage (V)
        Unit: 0.01 V
        """
        r = self.mb.read_holding_registers(self.id, self.REG_BUS_VOLT, 1)
        if not r:
            return None
        return r[0] * 0.01

    def read_driver_temperature(self):
        """
        Driver temperature (°C)
        Unit: 0.1 °C
        """
        r = self.mb.read_holding_registers(self.id, self.REG_DRIVER_TEMP, 1)
        if not r:
            return None
        return r[0] * 0.1
    
    def read_motor_temperature(self):
        """
        Motor temperature Left & Right (°C)

        Register 0x20A4:
        - High byte : Left motor temperature
        - Low byte  : Right motor temperature
        """
        r = self.mb.read_holding_registers(self.id, self.REG_MOTOR_TEMP, 1)
        if not r:
            return None, None

        val = r[0]
        temp_l = (val >> 8) & 0xFF
        temp_r = val & 0xFF
        return temp_l, temp_r
    
    def read_error_code(self):
        """
        Read driver error code for left & right motor
        Returns: (err_left, err_right)
        """

        err_l = self.mb.read_holding_registers(self.id, self.REG_ERROR_CODE_L,1)
        err_r = self.mb.read_holding_registers(self.id, self.REG_ERROR_CODE_R,1)


        if err_l is None or err_r is None:
            raise RuntimeError("Failed to read error code register")

        # jika return list
        if isinstance(err_l, list):
            err_l = err_l[0]
            err_r = err_r[0]

        return err_l, err_r

    def set_accel_profile(
        self,
        accel_ms=1000,
        decel_ms=800,
        quick_stop_ms=10
    ):
        """
        Set S-curve acceleration & deceleration time in ZLAC driver.
        Unit: milliseconds (U16)

        Uses Modbus write_single_register(slave, address, value)
        """

        # --- Acceleration time ---
        self.mb.write_single_register(self.id, 0x2080, int(accel_ms))  # Left accel
        self.mb.write_single_register(self.id, 0x2081, int(accel_ms))  # Right accel

        # --- Deceleration time ---
        self.mb.write_single_register(self.id, 0x2082, int(decel_ms))  # Left decel
        self.mb.write_single_register(self.id, 0x2083, int(decel_ms))  # Right decel

        # --- Quick stop (emergency stop only) ---
        self.mb.write_single_register(self.id, 0x2084, int(quick_stop_ms))
        self.mb.write_single_register(self.id, 0x2085, int(quick_stop_ms))



    # =====================================================
    # FAULT
    # =====================================================
    def read_fault(self):
        r = self.mb.read_holding_registers(self.id, self.REG_FAULT_CODE, 1)
        if not r:
            return None
        return r[0]
