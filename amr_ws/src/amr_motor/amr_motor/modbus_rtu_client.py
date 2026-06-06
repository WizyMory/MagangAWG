import threading
import time
from queue import Queue, Empty
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
import serial


class ModbusRTUClient:
    def __init__(
        self,
        port,
        baudrate=9600,
        timeout=0.5,
        parity='N',
        stopbits=1,
        bytesize=8,
        reconnect_interval=2.0,
        max_errors=3
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize

        self.reconnect_interval = reconnect_interval
        self.max_errors = max_errors

        self.client = None
        self.lock = threading.Lock()
        self.queue = Queue(maxsize=100)

        self.connected = False
        self.reconnecting = False

        self.last_error = ''
        self.last_success_time = None
        self.error_count = 0
        self.last_reconnect_attempt = 0.0

        self._create_client()
        self._connect()

        self.worker = threading.Thread(
            target=self._process_queue,
            daemon=True
        )
        self.worker.start()

    # ===================== CLIENT =====================

    def _create_client(self):
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass

        self.client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            parity=self.parity,
            stopbits=self.stopbits,
            bytesize=self.bytesize
        )

    def _connect(self):
        try:
            self.connected = self.client.connect()
            if self.connected:
                self.last_error = ''
                self.error_count = 0
        except Exception as e:
            self.connected = False
            self.last_error = str(e)

    def _ensure_connection(self):
        if self.connected:
            return True

        now = time.time()
        if now - self.last_reconnect_attempt < self.reconnect_interval:
            return False

        self.reconnecting = True
        self.last_reconnect_attempt = now

        try:
            self._create_client()
            self.connected = self.client.connect()
            if self.connected:
                self.reconnecting = False
                self.last_error = ''
                self.error_count = 0
                return True
        except Exception as e:
            self.last_error = str(e)

        self.connected = False
        return False

    # ===================== QUEUE WORKER =====================

    def _process_queue(self):
        while True:
            try:
                func, response = self.queue.get(timeout=0.1)
            except Empty:
                continue

            result = None
            with self.lock:
                if self._ensure_connection():
                    try:
                        result = func()
                        self.last_success_time = time.time()
                        self.error_count = 0
                    except (ModbusException, serial.SerialException, OSError) as e:
                        self.last_error = str(e)
                        self.error_count += 1
                        self.connected = False

                        if self.error_count >= self.max_errors:
                            self._force_disconnect()
                else:
                    self.last_error = 'Serial not connected'

            if not response.full():
                response.put(result)

            self.queue.task_done()

    def _force_disconnect(self):
        try:
            if self.client:
                self.client.close()
        except Exception:
            pass
        self.connected = False

    def _execute(self, func, timeout=1.0):
        response = Queue(maxsize=1)
        self.queue.put((func, response))

        try:
            return response.get(timeout=timeout)
        except Empty:
            self.last_error = 'Modbus response timeout'
            self.connected = False
            self.error_count += 1
            return None

    # ===================== MODBUS FC =====================

    def read_holding_registers(self, slave, address, count):
        def _f():
            r = self.client.read_holding_registers(address, count=count, device_id=slave)
            if r.isError():
                raise ModbusException(r)
            return r.registers
        return self._execute(_f)

    def write_single_register(self, slave, address, value):
        def _f():
            r = self.client.write_register(address, value, device_id=slave)
            if r.isError():
                raise ModbusException(r)
            return True
        return self._execute(_f)

    def write_multiple_registers(self, slave, address, values):
        def _f():
            r = self.client.write_registers(address, values, device_id=slave)
            if r.isError():
                raise ModbusException(r)
            return True
        return self._execute(_f)

    # ===================== DIAGNOSTIC =====================

    def get_status(self):
        level = 'OK'
        if not self.connected:
            level = 'ERROR'
        elif self.error_count > 0:
            level = 'WARN'

        return {
            'level': level,
            'connected': self.connected,
            'reconnecting': self.reconnecting,
            'port': self.port,
            'last_error': self.last_error,
            'error_count': self.error_count,
            'last_success_time': self.last_success_time
        }
