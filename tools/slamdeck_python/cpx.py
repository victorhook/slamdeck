from curses import def_prog_mode
from enum import IntEnum
import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass
from utils import BinaryPacket


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
class CPX_Routing(BinaryPacket):
    destination: CPX_Target
    source: CPX_Target
    function: CPX_Function
    lastPacket: bool = False

    def __len__(self) -> int:
        return 2

    def as_bytes(self) -> bytes:
        return struct.pack(
            'BB',
            (self.lastPacket << 6) | (self.source << 3) | self.destination,
            self.function
        )

@dataclass
class CPX_Packet(BinaryPacket):
    route: CPX_Routing
    data: BinaryPacket

    HEADER_SIZE: int = 2

    def as_bytes(self) -> bytes:
        d_type = type(self.data)
        if d_type is list:
            data = bytearray(self.data)
        elif d_type is bytes:
            data = self.data
        elif isinstance(self.data, BinaryPacket):
            data = self.data.as_bytes()
        else:
            raise RuntimeError(f'Failed to recognize data type: {d_type} for CPX payload.')

        return struct.pack('H', len(self.data)+2) + \
               self.route.as_bytes() + \
               data

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
