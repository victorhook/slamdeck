from dataclasses import dataclass
import logging
from threading import Thread, Event

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QPointF, QSizeF, QRectF, QObject
from PyQt5.QtWidgets import QMessageBox, QWidget, QGraphicsScene, QGraphicsView, QGraphicsTextItem
from PyQt5 import Qt
from PyQt5.QtGui import QPen, QColor, qRgb, QTextBlock
from pathlib import Path

import numpy as np

from tab import Tab
from slamdeck_python.slamdeck import Slamdeck
from slamdeck_python.backend_cpx import BackendCPX, BackendCPXProtocol
from slamdeck_python.backend import BackendParams, Callback
from slamdeck_python.backend_ip import BackendIp
from slamdeck_python.slamdeck import Sensor
from slamdeck_python.slamdeck_api import SlamdeckSensor
from slamdeck_python.utils import Subscriber

logger = logging.getLogger(__name__)

example_tab_class = uic.loadUiType(Path(__file__).parent.joinpath('ui/slamdeck_tab.ui'))[0]

import typing as t

from enum import IntEnum
class SensorDimension:
    DIMENSION_4X4: t.Tuple[int, int] = (4, 4)
    DIMENSION_8X8: t.Tuple[int, int] = (8, 8)


class SensorUi(QObject):

    GRID_SIZE = 80
    _s_distance = pyqtSignal(np.ndarray)

    def __init__(self, graphics: QGraphicsView, sensor: Sensor):
        QObject.__init__(self)
        self.graphics = graphics
        self.sensor = sensor
        self.dimesion: SensorDimension = SensorDimension.DIMENSION_4X4
        self.scene = QGraphicsScene()
        self.graphics.setScene(self.scene)
        self.pen = QPen(QColor(qRgb(172, 50, 99)))
        self.grids = self.create_grids()

        self._s_distance.connect(self._cb_distance)

        # Add subscriptions
        self.sensor.subscribe(self, 'data')

    def on_change(self, value: object) -> None:
        self._s_distance.emit(value)

    def _cb_distance(self, distances: np.ndarray) -> None:
        row, col = self.dimesion
        for r in range(row):
            for c in range(col):
                value = str(distances[r+c])
                self.grids[r][c].text.setPlainText(value)

    def create_grids(self):
        row, col = self.dimesion
        grids: t.List[t.List['Grid']] = []
        for r in range(row):
            grids.append([])
            for c in range(col):
                rect = QRectF(QPointF(r*self.GRID_SIZE, c*self.GRID_SIZE),
                           QSizeF(self.GRID_SIZE, self.GRID_SIZE))
                text = self.scene.addText(f'{r} {c}')
                self.scene.addRect(rect, self.pen)
                text.setPos(r*self.GRID_SIZE+self.GRID_SIZE/3, c*self.GRID_SIZE+self.GRID_SIZE/3)

                grids[r].append(self.Grid(rect, text))
        return grids

    @dataclass
    class Grid:
        grid: QRectF
        text: QGraphicsTextItem


class SlamdeckTab(Tab, example_tab_class):

    _s_connecting = pyqtSignal()
    _s_connected = pyqtSignal()
    _s_disconnected = pyqtSignal()
    _s_connection_error = pyqtSignal(str)

    def __init__(self, tabWidget, helper, *args):
        super(SlamdeckTab, self).__init__(*args)
        self.setupUi(self)
        self.tabWidget = tabWidget

        self.slamdeck = Slamdeck(
            BackendCPX(BackendIp(ip='192.168.1.73', port=5000))
        )

        self.sensor = SensorUi(self.graphics, self.slamdeck.get_sensor(SlamdeckSensor.BACK))


        # Attach button callbacks
        self.btnConnect.clicked.connect(self._connect)
        self.btnDisconnect.clicked.connect(self._disconnect)
        self.btnTest.clicked.connect(self._test)
        self.btnStartSampling.clicked.connect(self._start_sampling)

        # Attach callbacks for Slamdeck to trigger PyQt signals
        self.slamdeck.add_cb_connecting(Callback(self._s_connecting.emit))
        self.slamdeck.add_cb_connected(Callback(self._s_connected.emit))
        self.slamdeck.add_cb_connection_error(Callback(self._s_connection_error.emit))
        self.slamdeck.add_cb_disconnected(Callback(self._s_disconnected.emit))

        # Attach PyQt signals to callback functions
        self._s_connecting.connect(self._cb_connecting)
        self._s_connected.connect(self._cb_connected)
        self._s_disconnected.connect(self._cb_disconnected)
        self._s_connection_error.connect(self._cb_connection_error)

        self._sampling = Event()


    def _get_sensor_data_thread(self) -> None:
        import time
        update_frequency_ms = 1000
        sleep = update_frequency_ms * (1/1000)
        while self._sampling.is_set() and self.slamdeck.is_connected():
            self.slamdeck.get_data_from_sensor(SlamdeckSensor.BACK)

    def _start_sampling(self) -> None:
        self._sampling.set()
        Thread(target=self._get_sensor_data_thread, daemon=True).start()

    def _test(self) -> None:
        self.slamdeck.get_data_from_sensor(SlamdeckSensor.BACK)

    def _connect(self) -> None:
        if not self.slamdeck.is_connected():
            self.slamdeck.connect()

    def _disconnect(self) -> None:
        if self.slamdeck.is_connected():
            self.slamdeck.disconnect()

    def _cb_connecting(self):
        logger.debug('Connecting...')
        self.statusText.setText('Connecting')

    def _cb_connected(self):
        logger.debug('Connected')
        self.statusText.setText('Connected')

    def _cb_disconnected(self):
        logger.debug('Disconnected')
        self.statusText.setText('Disconnected')

    def _cb_connection_error(self, msg: str):
        logger.debug(f'Error: {msg}')
        self.statusText.setText(msg)

    def _logging_error(self, log_conf, msg):
        """Callback from the log layer when an error occurs"""

        QMessageBox.about(self, "Example error",
                          "Error when using log config"
                          " [{0}]: {1}".format(log_conf.name, msg))
