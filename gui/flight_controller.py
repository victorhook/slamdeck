from enum import IntEnum
import logging

from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

from gui import utils
from time import time


from PyQt5 import QtCore, QtWidgets

logger = logging.getLogger(__name__)

class FlyState(IntEnum):
    IDLE = 0
    STARTUP = 1
    FLYING = 2
    LANDING = 3

class FlightController(QtWidgets.QWidget):

    SPEED_FACTOR = 0.8
    STARTUP_TIME = 1

    def __init__(self, cf: Crazyflie, labels: list):
        super().__init__()
        self.cf = cf
        self.mc = MotionCommander(self.cf)
        self.labels = labels

        self.hover = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'z': 0.0}

        self.hoverTimer = utils.start_timer(self.sendHoverCommand, 100)
        self.update_labels = utils.start_timer(self._update_labels, 33)

        self.keyCB = self.updateHover

        self._state = FlyState.IDLE
        self.t0 = 0
        self.t1 = 0
        self._taken_off = False
        self._landed = False

    def start(self) -> None:
        if self._state == FlyState.IDLE:
            self._state = FlyState.STARTUP
            self.t0 = time()

    def stop(self) -> None:
        if self._state == FlyState.FLYING:
            self.t1 = time()
            self._state = FlyState.LANDING

    def _update_labels(self) -> None:
        self.labels['x'].setText(str(self.hover['x']))
        self.labels['y'].setText(str(self.hover['y']))
        self.labels['z'].setText(str(self.hover['z']))
        self.labels['yaw'].setText(str(self.hover['yaw']))
        self.labels['mode'].setText(self._state.name)

    def sendHoverCommand(self):
        if self._state == FlyState.IDLE:
            if self.cf.is_connected():
                self.cf.param.set_value('powerDist.idleThrust', 0)

        elif self._state == FlyState.LANDING:
            if not self._landed:
                self.cf.commander.send_hover_setpoint(0, 0, 0, 0)

            now = time()
            if (now - self.t1) > self.STARTUP_TIME:
                self._landed = True
                self._taken_off = False
                self._state = FlyState.IDLE

        elif self._state == FlyState.STARTUP:
            if not self._taken_off:
                if self.cf.is_connected():
                    #self.mc.take_off(0.1)
                    print('Sending power thrust!')
                    self.cf.param.set_value('powerDist.idleThrust', 20000)
                    self._taken_off = True
                    self._landed = False
                else:
                    print('Not connected!')
            now = time()
            if (now - self.t0) > self.STARTUP_TIME:
                self._state = FlyState.FLYING
        elif self._state == FlyState.FLYING:
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
                    self.start()
                elif self._state == FlyState.FLYING:
                    self.stop()
