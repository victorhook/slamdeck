from slamdeck_python.backend import Callback
from slamdeck_python.slamdeck import Slamdeck
from slamdeck_python.backend_cpx import BackendCPX
from slamdeck_python.backend_ip import BackendIp
from slamdeck_python.slamdeck_api import SlamdeckSensorId

IP = '192.168.6.83'
PORT = 5000
i = 0
STOP = 50

def new_data(*args):
    global i
    i += 1


if __name__ == '__main__':
    slamdeck = Slamdeck(BackendCPX(BackendIp(IP, PORT)))
    slamdeck.add_cb_on_new_data(Callback(new_data))
    slamdeck_thread = slamdeck.connect()

    while not slamdeck.is_connected():
        pass

    print('Connected!')
    slamdeck.start_sampling(SlamdeckSensorId.BACK)

    while i < 2:
        pass

    slamdeck.disconnect()
    slamdeck_thread.join()

