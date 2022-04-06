from tkinter import E
import unittest
from slamdeck_python.backend_cpx import BackendCPX, BackendIp
from slamdeck_python.slamdeck import Slamdeck, Callback

from threading import Event, Thread
import time

from slamdeck_python.slamdeck_api import SlamdeckSensor


class TestBandwidth(unittest.TestCase):

    slamdeck = None
    IP = '192.168.6.83'
    PORT = 5000
    running = Event()
    CONNECTION_TIMEOUT_MS = 1000
    t0: int
    slamdeck_thread: Thread = None
    update_freq = 10

    def setUp(self):
        self.slamdeck = Slamdeck(BackendCPX(BackendIp(self.IP, self.PORT)))
        self.slamdeck.add_cb_connected(Callback(self.start))
        self.slamdeck_thread = self.slamdeck.connect()
        self.t0 = time.time()

    def tearDown(self):
        if self.slamdeck.is_connected():
            self.slamdeck.disconnect()

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
        sensor_model = self.slamdeck.get_sensor_model(sensor)
        self.slamdeck.start_sampling(sensor)

        i = 0
        t0 = time.time()
        try:
            while True:
                now = time.time()
                dt = now - t0

                if dt > (1/self.update_freq):
                    t0 = now
                    grid = 0
                    for row in range(8):
                        for col in range(8):
                            print(sensor_model.data[grid], end=' ')
                            grid += 1
                        print('')
                    print('')

        except KeyboardInterrupt as e:
            self.slamdeck.stop_sampling()
            self.slamdeck.disconnect()


        self.slamdeck_thread.join()


if __name__ == '__main__':
    unittest.main()