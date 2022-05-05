from enum import IntEnum
import logging
from vispy import scene

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

from gui import utils
from slamdeck_python.utils import CrazyflieModel
from time import time


from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import pyqtSignal
from env.bin.activate_this import bin_dir

logger = logging.getLogger(__name__)

class FlyState(IntEnum):
    IDLE = 0
    STARTUP = 1
    FLYING = 2
    LANDING = 3

class FlightController(QtWidgets.QWidget):

    SPEED_FACTOR = 0.3

    def __init__(self, cf: Crazyflie, labels: list):
        super().__init__()
        self.cf = cf
        self.labels = labels

        self.hover = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'z': 0.0}

        self.hoverTimer = utils.start_timer(self.sendHoverCommand, 100)
        self.update_labels = utils.start_timer(self._update_labels, 33)
        #self.hoverTimer.start()

        #self.canvas.on_key_press = self.on_key_press
        #self.canvas.on_key_release = self.on_key_release

        #self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.keyCB = self.updateHover

        self._state = FlyState.IDLE
        self.t0 = 0
        self._sent = False

    def _update_labels(self) -> None:
        self.labels['x'].setText(str(self.hover['x']))
        self.labels['y'].setText(str(self.hover['y']))
        self.labels['z'].setText(str(self.hover['z']))
        self.labels['yaw'].setText(str(self.hover['yaw']))

    def sendHoverCommand(self):
        if self._state == FlyState.IDLE:
            self.cf.commander.send_stop_setpoint()
        elif self._state == FlyState.STARTUP:
            if not self._sent:
                #self.cf.param.set_value('powerDist.idleThrust', 20000)
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
                self.hover['z']
            )

    def updateHover(self, k, v):
        if (k != 'z'):
            self.hover[k] = v * self.SPEED_FACTOR
        else:
            self.hover[k] += v
            self.hover[k] = max(0, self.hover[k])

        self.hover[k] = round(self.hover[k], 3)

        self._update_labels()

    def on_key_press(self, event):
        if (not event.isAutoRepeat()):
            if (event.key() == QtCore.Qt.Key_Left):
                self.keyCB('y', 1)
            if (event.key() == QtCore.Qt.Key_Right):
                self.keyCB('y', -1)
            if (event.key() == QtCore.Qt.Key_Up):
                self.keyCB('x', 1)
            if (event.key() == QtCore.Qt.Key_Down):
                self.keyCB('x', -1)
            if (event.key() == QtCore.Qt.Key_A):
                self.keyCB('yaw', -70)
            if (event.key() == QtCore.Qt.Key_D):
                self.keyCB('yaw', 70)
            if (event.key() == QtCore.Qt.Key_Z):
                self.keyCB('yaw', -200)
            if (event.key() == QtCore.Qt.Key_X):
                self.keyCB('yaw', 200)
            if (event.key() == QtCore.Qt.Key_W):
                self.keyCB('z', 0.1)
            if (event.key() == QtCore.Qt.Key_S):
                self.keyCB('z', -0.1)
            if (event.key() == QtCore.Qt.Key_F):
                #self._flying = not self._flying
                pass

    def on_key_release(self, event):
        if (not event.isAutoRepeat()):
            if (event.key() == QtCore.Qt.Key_Left):
                self.keyCB('y', 0)
            if (event.key() == QtCore.Qt.Key_Right):
                self.keyCB('y', 0)
            if (event.key() == QtCore.Qt.Key_Up):
                self.keyCB('x', 0)
            if (event.key() == QtCore.Qt.Key_Down):
                self.keyCB('x', 0)
            if (event.key() == QtCore.Qt.Key_A):
                self.keyCB('yaw', 0)
            if (event.key() == QtCore.Qt.Key_D):
                self.keyCB('yaw', 0)
            if (event.key() == QtCore.Qt.Key_W):
                self.keyCB('z', 0)
            if (event.key() == QtCore.Qt.Key_S):
                self.keyCB('z', 0)
            if (event.key() == QtCore.Qt.Key_Z):
                self.keyCB('yaw', 0)
            if (event.key() == QtCore.Qt.Key_X):
                self.keyCB('yaw', 0)
            if (event.key() == QtCore.Qt.Key_F):
                if self._state == FlyState.IDLE:
                    self._state = FlyState.STARTUP
                    self.t0 = time()
                elif self._state == FlyState.FLYING:
                    self._state = FlyState.IDLE
