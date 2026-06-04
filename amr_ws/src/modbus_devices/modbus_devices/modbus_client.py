#from asyncio import timeout
import threading
import time
from queue import Queue, Empty
from urllib import response

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException


class ModbusRTUClient:
    def __init__(
        self,
        port,
        baudrate=9600,
        timeout=0.5,
        parity='N',
        stopbits=1,
        bytesize=8,
        reconnect_interval=2.0
    ):
        self.port = port
        self.reconnect_interval = reconnect_interval

        self.client = ModbusSerialClient(
            method='rtu',
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize
        )

        self.lock = threading.Lock()
        self.queue = Queue()

        self.connected = False
        self.last_error = ''
        self.last_success_time = None

        self.worker = threading.Thread(
            target=self._process_queue,
            daemon=True
        )
        self.worker.start()

        self._connect()

    # ===================== CONNECTION =====================

    def _connect(self):
        self.connected = self.client.connect()
        if not self.connected:
            self.last_error = 'Connect failed'

    def _ensure_connection(self):
        if not self.connected:
            self._connect()
            if not self.connected:
                time.sleep(self.reconnect_interval)
                return False
        return True

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
                        self.last_error = ''
                    except Exception as e:
                        self.last_error = str(e)
                        self.connected = False

            if not response.full():
                response.put(result)

            self.queue.task_done()

    def _execute(self, func, timeout=1.0):
        response = Queue(maxsize=1)
        self.queue.put((func, response))

        try:
            return response.get(timeout=timeout)
        except Empty:
            self.last_error = 'Modbus response timeout'
            self.connected = False
            return None

    def _execute(self, func, timeout=1.0):
        response = Queue(maxsize=1)
        self.queue.put((func, response))

        try:
            return response.get(timeout=timeout)
        except Empty:
            self.last_error = 'Modbus response timeout'
            self.connected = False
            return None

    # ===================== MODBUS FC =====================

    # FC01
    def read_coils(self, slave, address, count):
        def _f():
            r = self.client.read_coils(address, count, slave=slave)
            if r.isError():
                raise ModbusException(r)
            return r.bits[:count]
        return self._execute(_f)

    # FC02
    def read_discrete_inputs(self, slave, address, count):
        def _f():
            r = self.client.read_discrete_inputs(address, count, slave=slave)
            if r.isError():
                raise ModbusException(r)
            return r.bits[:count]
        return self._execute(_f)

    # FC03
    def read_holding_registers(self, slave, address, count):
        def _f():
            r = self.client.read_holding_registers(address, count, slave=slave)
            if r.isError():
                raise ModbusException(r)
            return r.registers
        return self._execute(_f)

    # FC04
    def read_input_registers(self, slave, address, count):
        def _f():
            r = self.client.read_input_registers(address, count, slave=slave)
            if r.isError():
                raise ModbusException(r)
            return r.registers
        return self._execute(_f)

    # FC05
    def write_single_coil(self, slave, address, value):
        def _f():
            r = self.client.write_coil(address, value, slave=slave)
            if r.isError():
                raise ModbusException(r)
            return True
        return self._execute(_f)

    # FC06
    def write_single_register(self, slave, address, value):
        def _f():
            r = self.client.write_register(address, value, slave=slave)
            if r.isError():
                raise ModbusException(r)
            return True
        return self._execute(_f)

    # FC15
    def write_multiple_coils(self, slave, address, values):
        def _f():
            r = self.client.write_coils(address, values, slave=slave)
            if r.isError():
                raise ModbusException(r)
            return True
        return self._execute(_f)

    # FC16
    def write_multiple_registers(self, slave, address, values):
        def _f():
            r = self.client.write_registers(address, values, slave=slave)
            if r.isError():
                raise ModbusException(r)
            return True
        return self._execute(_f)

    # ===================== DIAGNOSTIC =====================

    def get_status(self):
        return {
            'connected': self.connected,
            'port': self.port,
            'last_error': self.last_error,
            'last_success_time': self.last_success_time
        }
