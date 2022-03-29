import unittest
from slamdeck_python.backend_cpx import BackendCPX, BackendIp
from slamdeck_python.slamdeck import Slamdeck, Callback

from threading import Event, Thread
import time

from slamdeck_python.slamdeck_api import SlamdeckSensor


class TestBandwidth(unittest.TestCase):

    slamdeck = None
    IP = '192.168.1.73'
    PORT = 5000
    running = Event()
    CONNECTION_TIMEOUT_MS = 1000
    t0: int
    slamdeck_thread: Thread = None

    def setUp(self):
        self.slamdeck = Slamdeck(BackendCPX(BackendIp(self.IP, self.PORT)))
        self.slamdeck.add_cb_connected(Callback(self.start))
        self.slamdeck_thread = self.slamdeck.connect()
        self.t0 = time.time()

    def tearDown(self):
        pass

    def start(self):
        self.running.set()

    def is_ready(self) -> bool:
        dt = (time.time() - self.t0) * 1000
        if dt > self.CONNECTION_TIMEOUT_MS:
            raise RuntimeError('Timeout reached on connection attempt.')
        return self.running.is_set()

    def on_change(self, data):
        print('New data:', end='')
        print(data)
        self.slamdeck.disconnect()

    def get_sensor_info(self, sensor: SlamdeckSensor) -> dict:
        self.slamdeck.get_data_from_sensor(sensor)

    def testBw(self):
        while not self.is_ready():
            pass

        print('Connected!')
        sensor = SlamdeckSensor.BACK
        model = self.slamdeck.get_sensor(sensor)
        model.subscribe(self, 'data')

#        while True:
        self.slamdeck.get_data_from_sensor(sensor)
 #           time.sleep(.5)

        self.slamdeck_thread.join()


if __name__ == '__main__':
    unittest.main()