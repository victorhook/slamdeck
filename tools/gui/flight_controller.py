from enum import IntEnum
import logging
from vispy import scene

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

from slamdeck_python.utils import CrazyflieModel
from time import time

try:
    from sip import setapi
    setapi('QVariant', 2)
    setapi('QString', 2)
except ImportError:
    pass

from PyQt5 import QtCore, QtWidgets


logger = logging.getLogger(__name__)

class FlyState(IntEnum):
    IDLE = 0
    STARTUP = 1
    FLYING = 2
    LANDING = 3

class FlightController:

    SPEED_FACTOR = 0.3

    def __init__(self, cf: Crazyflie, canvas: scene.SceneCanvas):
        self.cf = cf
        self.canvas = canvas

        self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.0}

        self.hoverTimer = QtCore.QTimer()
        self.hoverTimer.timeout.connect(self.sendHoverCommand)
        self.hoverTimer.setInterval(100)
        self.hoverTimer.start()

        self.canvas.on_key_press = self.on_key_press
        self.canvas.on_key_release = self.on_key_release

        self.keyCB = self.updateHover

        self._state = FlyState.IDLE
        self.t0 = 0
        self._sent = False

    def sendHoverCommand(self):
        print(self._state, self._sent)
        if self._state == FlyState.IDLE:
            self.cf.commander.send_stop_setpoint()
        elif self._state == FlyState.STARTUP:
            if not self._sent:
                self.cf.param.set_value('powerDist.idleThrust', 20000)
                self._sent = True
            now = time()
            if (now - self.t0) > 1:
                self._state = FlyState.FLYING
        elif self._state == FlyState.FLYING:
            return
            self.cf.commander.send_hover_setpoint(
                self.hover['x'],
                self.hover['y'],
                self.hover['yaw'],
                self.hover['height']
            )

    def updateHover(self, k, v):
        if (k != 'height'):
            self.hover[k] = v * self.SPEED_FACTOR
        else:
            self.hover[k] += v

    def on_key_press(self, event):
        if (not event.native.isAutoRepeat()):
            if (event.native.key() == QtCore.Qt.Key_Left):
                self.keyCB('y', 1)
            if (event.native.key() == QtCore.Qt.Key_Right):
                self.keyCB('y', -1)
            if (event.native.key() == QtCore.Qt.Key_Up):
                self.keyCB('x', 1)
            if (event.native.key() == QtCore.Qt.Key_Down):
                self.keyCB('x', -1)
            if (event.native.key() == QtCore.Qt.Key_A):
                self.keyCB('yaw', -70)
            if (event.native.key() == QtCore.Qt.Key_D):
                self.keyCB('yaw', 70)
            if (event.native.key() == QtCore.Qt.Key_Z):
                self.keyCB('yaw', -200)
            if (event.native.key() == QtCore.Qt.Key_X):
                self.keyCB('yaw', 200)
            if (event.native.key() == QtCore.Qt.Key_W):
                self.keyCB('height', 0.1)
            if (event.native.key() == QtCore.Qt.Key_S):
                self.keyCB('height', -0.1)
            if (event.native.key() == QtCore.Qt.Key_F):
                #self._flying = not self._flying
                pass

    def on_key_release(self, event):
        if (not event.native.isAutoRepeat()):
            if (event.native.key() == QtCore.Qt.Key_Left):
                self.keyCB('y', 0)
            if (event.native.key() == QtCore.Qt.Key_Right):
                self.keyCB('y', 0)
            if (event.native.key() == QtCore.Qt.Key_Up):
                self.keyCB('x', 0)
            if (event.native.key() == QtCore.Qt.Key_Down):
                self.keyCB('x', 0)
            if (event.native.key() == QtCore.Qt.Key_A):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_D):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_W):
                self.keyCB('height', 0)
            if (event.native.key() == QtCore.Qt.Key_S):
                self.keyCB('height', 0)
            if (event.native.key() == QtCore.Qt.Key_Z):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_X):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key_F):
                if self._state == FlyState.IDLE:
                    self._state = FlyState.STARTUP
                    self.t0 = time()
                elif self._state == FlyState.FLYING:
                    self._state = FlyState.IDLE
