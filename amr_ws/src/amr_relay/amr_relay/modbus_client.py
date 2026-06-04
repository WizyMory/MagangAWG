from pymodbus.client import ModbusSerialClient


class ModbusRelayClient:
    """
    Modbus RTU Relay Client
    ----------------------
    - FC01 : Read relay status
    - FC05 : Write single relay
    - FC0F : Write multiple relays
    """

    def __init__(self, port, baudrate, slave_id):
        self.slave_id = slave_id
        self.client = ModbusSerialClient(
            method='rtu',
            port=port,
            baudrate=baudrate,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=1
        )

        if not self.client.connect():
            raise RuntimeError('Modbus RTU connection failed')

    # =================================================
    # READ RELAY STATUS (FC=0x01)
    # =================================================
    def read_relays(self, start_addr, count):
        rr = self.client.read_coils(
            address=start_addr,
            count=count,
            slave=self.slave_id
        )
        if rr.isError():
            raise RuntimeError('Read coils failed')
        return rr.bits[:count]

    # =================================================
    # WRITE SINGLE RELAY (FC=0x05)
    # =================================================
    def set_relay(self, addr, state: bool):
        wr = self.client.write_coil(
            address=addr,
            value=state,
            slave=self.slave_id
        )
        if wr.isError():
            raise RuntimeError('Write coil failed')

    # =================================================
    # WRITE MULTIPLE RELAYS (FC=0x0F)
    # =================================================
    def set_relays(self, start_addr, states):
        wr = self.client.write_coils(
            address=start_addr,
            values=states,
            slave=self.slave_id
        )
        if wr.isError():
            raise RuntimeError('Write multiple coils failed')

    # =================================================
    # ALL ON / OFF
    # =================================================
    def all_on(self, start_addr, count):
        self.set_relays(start_addr, [True] * count)

    def all_off(self, start_addr, count):
        self.set_relays(start_addr, [False] * count)

    # =================================================
    # CLEANUP
    # =================================================
    def close(self):
        self.client.close()
