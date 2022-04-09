from enum import IntEnum
import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass

from slamdeck_python.transport import Transport


class CPX_Target(IntEnum):
  STM32 = 1  # The STM in the Crazyflie
  ESP32 = 2  # The ESP on the AI-deck
  HOST = 3   # A remote computer connected via Wifi
  GAP8 = 4   # The GAP8 on the AI-deck

class CPX_Function(IntEnum):
  SYSTEM = 1
  CONSOLE = 2
  CRTP = 3
  WIFI_CTRL = 4
  APP = 5
  TEST = 0x0E
  BOOTLOADER = 0x0F


@dataclass
class CPX_Routing:
    destination: CPX_Target
    source: CPX_Target
    function: CPX_Function
    lastPacket: bool = False

    def as_bytes(self) -> bytes:
        return struct.pack(
            'BB',
            (self.lastPacket << 6) | (self.source << 3) | self.destination,
            self.function
        )

@dataclass
class CPX_Packet:
    route: CPX_Routing
    data: bytes

    HEADER_SIZE: int = 2

    def as_bytes(self) -> bytes:
        return struct.pack('H', len(self.data)+self.HEADER_SIZE) + \
               self.route.as_bytes() + \
               self.data

    def __len__(self) -> int:
        return len(self.route) + len(self.data)

    def __str__(self):
        return f'From: {self.route.destination.name}\n' \
               f'To: {self.route.source.name}\n' \
               f'Function: {self.route.function.name}\n' \
               f'Data len: {len(self.data)}\n' \
               f'Data: \n{self.data}'

    @staticmethod
    def from_bytes(data: bytes) -> 'CPX_Packet':
        cpx_header = struct.unpack('BB', data[:2])
        return CPX_Packet(
            CPX_Routing(
                destination=CPX_Target(cpx_header[0] & 0b111),
                source=CPX_Target((cpx_header[0] & (0b111 << 3) ) >> 3),
                function=CPX_Function(cpx_header[1])
            ),
            data=data[2:]
        )


class BackendCPX:

    def __init__(self,
                 transport: Transport,
                 route: CPX_Routing = CPX_Routing(
                     destination=CPX_Target.ESP32,
                     source=CPX_Target.HOST,
                     function=CPX_Function.APP
                 )
            ) -> None:
        super().__init__()
        self._transport = transport
        self._route = route

    def connect(self) -> bool:
        return self._transport.connect()

    def disconnect(self) -> bool:
        return self._transport.disconnect()

    def write(self, data: bytes) -> int:
        return self._transport.write(
            CPX_Packet(route=self._route, data=data).as_bytes()
        )

    def read(self) -> bytes:
        first_two_bytes = self._transport.read(2)
        if not first_two_bytes:
            return

        # First 2 bytes of CPX wifi packet is the packet payload length.
        size = struct.unpack('H', first_two_bytes)[0]
        # Next 2 bytes is the CPX header, and the remaining is the app data.
        data = self._transport.read(size)

        if not data:
            return

        return data[CPX_Packet.HEADER_SIZE:]
