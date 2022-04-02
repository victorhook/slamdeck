from dataclasses import dataclass
from enum import IntEnum
import struct

from slamdeck_python.cpx import BinaryPacket, CPX_Packet


""" Slamdeck API commands """

class SlamdeckCommand(IntEnum):
    GET_SENSOR_STATUS = 0
    GET_DATA_FROM_SENSOR = 1
    GET_I2C_ADDRESS = 2
    SET_I2C_ADDRESS = 3
    GET_POWER_MODE = 4
    SET_POWER_MODE = 5
    GET_RESOLUTION = 6
    SET_RESOLUTION = 7
    GET_RANGING_FREQUENCY_HZ = 8
    SET_RANGING_FREQUENCY_HZ = 9
    GET_INTEGRATION_TIME_MS = 10
    SET_INTEGRATION_TIME_MS = 11
    GET_SHARPENER_PERCENT = 12
    SET_SHARPENER_PERCENT = 13
    GET_TARGET_ORDER = 14
    SET_TARGET_ORDER = 15
    GET_RANGING_MODE = 16
    SET_RANGING_MODE = 17

class SlamdeckResponse(IntEnum):
    RESULT_OK = 0

class SlamdeckSensor(IntEnum):
    MAIN    = 0
    FRONT   = 1
    RIGHT   = 2
    BACK    = 3
    LEFT    = 4
    NOT_SET = 0xff


class VL53L5CX_Status(IntEnum):
    SENSOR_FAILED      = 0
    SENSOR_NOT_ENABLED = 1
    SENSOR_OK          = 2

class VL53L5CX_PowerMode(IntEnum):
    POWER_MODE_SLEEP  = 0
    POWER_MODE_WAKEUP = 1

class VL53L5CX_Resolution(IntEnum):
    RESOLUTION_4X4 = 16
    RESOLUTION_8X8 = 64

class VL53L5CX_TargetOrder(IntEnum):
    TARGET_ORDER_CLOSEST   = 1
    TARGET_ORDER_STRONGEST = 2

class VL53L5CX_RangingMode(IntEnum):
    RANGING_MODE_CONTINUOUS = 1
    RANGING_MODE_AUTONOMOUS = 3


@dataclass
class SlamdeckApiPacket(BinaryPacket):
    command: SlamdeckCommand
    sensor: SlamdeckSensor
    data: int = 0

    HEADER_SIZE = 2

    def __str__(self) -> str:
        return f'  Command: {self.command.name} \n' \
               f'  Sensor: {self.sensor.name} \n' \
               f'  Data: {self.data}'

    def __len__(self) -> int:
        return 2

    def as_bytes(self) -> bytes:
        return struct.pack(
                   'BB',
                   self.command.value | (self.sensor.value << 5),
                   self.data
                )

def pretty(packet: bytes):
    print(' '.join(f'{a:02x}      ' for a in packet))
    print(' '.join(f'{bin(a)[2:].zfill(8)}' for a in packet))

def ashex(data: bytes) -> str:
    return ' '.join([f'{a:02x}' for a in data])



def test():
    import struct
    import socket
    from cpx import CPX_Routing, CPX_Target, CPX_Function
    sock = socket.socket()
    sock.connect(('192.168.1.73', 5000))
    print('Connected')

    packet = SlamdeckApiPacket(
        command=SlamdeckCommand.GET_DATA_FROM_SENSOR,
        sensor=SlamdeckSensor.BACK
    )

    cpx = CPX_Packet(
        route=CPX_Routing(
            destination=CPX_Target.ESP32,
            source=CPX_Target.HOST,
            function=CPX_Function.APP
        ),
        data=packet
    )

    print('Sending\n--------------------------------------------')
    print(cpx)
    print('\n')
    pretty(cpx.as_bytes())
    sock.send(cpx.as_bytes())
    print('----------------------------------------------')

    size = struct.unpack('H', sock.recv(2))[0]
    print(f'Received, size: {size} bytes\n--------------------------------------')
    resp = sock.recv(size)

    # Helper function to recv n bytes or return None if EOF is hit
    cpx = CPX_Packet.from_bytes(resp)
    print(resp)
    print('------------------------------------\n')
    print(cpx)
    print('----------------------------------')

    sock.close()


if __name__ == '__main__':
    test()
