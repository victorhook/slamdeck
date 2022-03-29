import unittest
from slamdeck_python.backend_cpx import BackendCPX, BackendIp
from slamdeck_python.cpx import CPX_Function, CPX_Packet, CPX_Routing, CPX_Target
from slamdeck_python.slamdeck import Slamdeck, Callback

from threading import Event, Thread
import time

from slamdeck_python.slamdeck_api import SlamdeckCommand, SlamdeckSensor, SlamdeckApiPacket
import socket
import struct

class TestBandwidth(unittest.TestCase):

    IP = '192.168.1.73'
    PORT = 5000

    def setUp(self):
        self.sock = socket.socket()
        self.sock.connect((self.IP, self.PORT))

    def tearDown(self):
        self.sock.close()

    def tttest_socket(self):
        packet = CPX_Packet(
            CPX_Routing(CPX_Target.ESP32, CPX_Target.HOST, CPX_Function.APP),
            data=SlamdeckApiPacket(SlamdeckCommand.GET_SENSOR_STATUS, SlamdeckSensor.BACK)
            )
        self.sock.send(packet.as_bytes())
        size = struct.unpack('H', self.sock.recv(2))[0]
        packet = self.sock.recv(size)
        return size + 2

    def test_one(self):
        #self.tearDown()
        #self.setUp()
        i = 0
        try:
            data = 0
            t0 = time.time()
            while True:
                data += self.tttest_socket()
                print(i)
                i += 1

                t1 = time.time()
                if (t1 - t0) > 1:
                    print(data)
                    data = 0
                    t0 = t1

        finally:
            self.tearDown()

if __name__ == '__main__':
    unittest.main()