import unittest
from slamdeck_python.backend_cpx import BackendCPX, BackendIp
from slamdeck_python.slamdeck import Slamdeck, Callback

from threading import Event, Thread
import time
from queue import Queue

from slamdeck_python.slamdeck_api import SlamdeckSensorId


class TestBandwidth(unittest.TestCase):

    slamdeck = None
    IP = '192.168.6.83'
    PORT = 5000
    running = Event()
    CONNECTION_TIMEOUT_MS = 1000
    t0: int
    slamdeck_thread: Thread = None
    commands = Queue()
    datapackets = 0
    MAX_DATA_PACKETS = 300
    last_data = 0
    packets = 0
    PACKET_SIZE = 640

    def setUp(self):
        self.slamdeck = Slamdeck(BackendCPX(BackendIp(self.IP, self.PORT)))
        self.slamdeck.add_cb_connected(Callback(self.start))
        self.slamdeck_thread = self.slamdeck.connect()
        self.t0 = time.time()
        self.done = False

    def tearDown(self):
        pass

    def start(self):
        self.running.set()

    def on_change(self, data):
        print('New data:', end='')
        print(data)
        self.slamdeck.disconnect()

    def on_change_all(self, attr: str, data):
        #if attr == 'target_order':
            #print(self.slamdeck.get_sensor_model(SlamdeckSensorId.ALL))

        if attr == 'data':
            if self.last_data == 0:
                self.last_data = time.time()

            now = time.time()
            dt = now - self.last_data
            if dt > 1:
            #print([hex(i)[2:].zfill(2) for i in data])
                print(f'Data packets per second: {self.packets}: {self.packets*self.PACKET_SIZE} bytes')
                self.packets = 0
                self.last_data = now

            self.packets += 1
            self.datapackets += 1
            if self.datapackets >= self.MAX_DATA_PACKETS:
                self.slamdeck.disconnect()


    def get_sensor_info(self, sensor: SlamdeckSensorId) -> dict:
        sensor = SlamdeckSensorId.ALL
        #self.slamdeck.get_sharpener_percent(sensor)
        #self.slamdeck.get_i2c_address(sensor)
        #self.slamdeck.get_integration_time_ms(sensor)
        #self.slamdeck.get_power_mode(sensor)
        #self.slamdeck.get_ranging_frequency_hz(sensor)
        #self.slamdeck.get_ranging_mode(sensor)
        #self.slamdeck.get_resolution(sensor)
        #self.slamdeck.get_target_order(sensor)

        for i in range(self.MAX_DATA_PACKETS):
            self.slamdeck.get_data_from_sensor(sensor)

    def testBw(self):
        while not self.slamdeck.is_connected():
            pass

        print('Connected!')
        sensor = SlamdeckSensorId.BACK
        sensor_model = self.slamdeck.get_sensor_model(sensor)
        sensor_model.subscribe_to_all(self)

        self.get_sensor_info(sensor)

        self.slamdeck_thread.join()


if __name__ == '__main__':
    unittest.main()