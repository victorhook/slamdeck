from threading import currentThread
from xml.dom.expatbuilder import theDOMImplementation
from backend import Backend
from backend_ip import BackendIp
from backend_serial import BackendSerial
from enum import IntEnum
from cpx import CPX_Function, CPX_Packet, CPX_Routing, CPX_Target
from slamdeck_api import SlamdeckApiPacket, SlamdeckCommand, SlamdeckSensor
from utils import BinaryPacket, SimplePacket
import struct


class BackendCPXProtocol(IntEnum):
    TCP  = 1
    UART = 2


class BackendCPX(Backend):

    def __init__(self,
                 backend_protocol: BackendCPXProtocol,
                 backend_args: dict,
                 route: CPX_Routing = CPX_Routing(
                     destination=CPX_Target.ESP32,
                     source=CPX_Target.HOST,
                     function=CPX_Function.APP
                 )
            ) -> None:
        super().__init__()
        self._backend_protocol: BackendCPXProtocol = backend_protocol
        self._backend: Backend = self._get_backend(backend_args)
        self._route = route

    def _get_backend(self, backend_args: dict) -> Backend:
        if self._backend_protocol == BackendCPXProtocol.TCP:
            backend = BackendIp(**backend_args)
        elif self._backend_protocol == BackendCPXProtocol.UART:
            backend = BackendSerial(**backend_args)
        else:
            raise RuntimeError(f'Failed to initialize backend {self._backend_protocol.name}')

        return backend

    def do_start(self) -> bool:
        return self._backend.do_start()

    def do_stop(self) -> bytes:
        return self._backend.do_stop()

    def do_write(self, data: bytes) -> int:
        return self._backend.do_write(
            CPX_Packet(route=self._route, data=data).as_bytes()
        )

    def do_read(self, size: int) -> bytes:
        # First 2 bytes of CPX wifi packet is the packet payload length.
        size = struct.unpack('H', self._backend.do_read(2))[0]
        # Rest of packet is the payload.
        data = self._backend.do_read(size)
        return data



if __name__ == '__main__':
    import time
    import logging
    logging.basicConfig(level=logging.INFO)
    b = BackendCPX(BackendCPXProtocol.TCP, {'ip': '192.168.1.73', 'port': 5000})
    t0 = 0
    dts = 0
    samples = []
    avgs = 5

    def on_complete(data: bytes):
        global t0, dts, avgs
        t1 = time.time()
        if t0 == 0:
            t0 = time.time()
        dt = t1 - t0
        t0 = t1

        samples.insert(0, dt)

        if len(samples) == avgs:
            samples.pop()

        avg = (sum(samples) / avgs) * 1000

        packet = CPX_Packet.from_bytes(data)
        data = struct.unpack('HHHHHHHHHHHHHHHH', packet.data)
        print(f'----------------- {avg:4} ms -------------------------')
        print(f' {data[0]:3} {data[1]:3} {data[2]:3} {data[3]:3} ')
        print(f' {data[4]:3} {data[5]:3} {data[6]:3} {data[7]:3} ')
        print(f' {data[8]:3} {data[9]:3} {data[10]:3} {data[11]:3} ')
        print(f' {data[12]:3} {data[13]:3} {data[14]:3} {data[15]:3} ')
        print('-------------------------------------------------')

    def read_thread(delay_ms):
        while True:
            b.write(SlamdeckApiPacket(
                SlamdeckCommand.GET_DATA_FROM_SENSOR,
                SlamdeckSensor.BACK
            ), 36, on_complete)
            time.sleep(delay_ms/1000)

    b.start()
    while not b.is_connected():
        pass

    read_thread(10)

    """
    while True:
        cmd = input('> ')
        if cmd == 'go':
            print('-------------------------------------------------')
            b.start()
            while not b.is_connected():
                pass
            b.write(SlamdeckApiPacket(
                SlamdeckCommand.GET_DATA_FROM_SENSOR,
                SlamdeckSensor.BACK
                ), 36, on_complete)
    """

    b.stop()
