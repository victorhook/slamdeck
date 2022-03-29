from dataclasses import dataclass
import logging
from threading import Thread, Event

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QPointF, QSizeF, QRectF, QObject
from PyQt5.QtWidgets import QMessageBox, QWidget, QGraphicsScene, QGraphicsView, QGraphicsTextItem, QGraphicsRectItem, QComboBox, QLabel, QLineEdit, QPushButton
from PyQt5 import Qt
from PyQt5.QtGui import QPen, QColor, qRgb, QTextBlock, QBrush
from pathlib import Path

import numpy as np
from  matplotlib.colors import LinearSegmentedColormap, rgb2hex


from tab import Tab
from slamdeck_python.slamdeck import Slamdeck
from slamdeck_python.backend_cpx import BackendCPX, BackendCPXProtocol
from slamdeck_python.backend import BackendParams, Callback
from slamdeck_python.backend_ip import BackendIp
from slamdeck_python.slamdeck import Sensor
from slamdeck_python.slamdeck_api import SlamdeckSensor, VL53L5CX_PowerMode, VL53L5CX_RangingMode, VL53L5CX_Resolution, VL53L5CX_TargetOrder
from slamdeck_python.utils import Subscriber

logger = logging.getLogger(__name__)

example_tab_class = uic.loadUiType(Path(__file__).parent.joinpath('ui/slamdeck_tab.ui'))[0]

import typing as t
from enum import IntEnum
import time

t0 = 0
received = False

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
        self.cmap = LinearSegmentedColormap.from_list('rg',["r", "w", "g"], N=500)
        self.grids = self.create_grids()

        self._s_distance.connect(self._cb_distance)

        # Add subscriptions
        self.sensor.subscribe(self, 'data')

    def on_change(self, value: np.ndarray) -> None:
        self._s_distance.emit(value)

    def _cb_distance(self, distances: np.ndarray) -> None:
        global t0, received
        t1 = time.time()
        dt = t1 - t0
        print(dt)

        received = True

        row, col = self.dimesion
        for r in range(row):
            for c in range(col):
                grid = self.grids[r][c]
                value = distances[r+c]
                grid.rect.setBrush(QBrush(QColor(rgb2hex(self.cmap(value)))))
                grid.text.setPlainText(str(value))

    def create_grids(self) -> t.List[t.List['Grid']]:
        row, col = self.dimesion
        grids: t.List[t.List['Grid']] = []
        for r in range(row):
            grids.append([])
            for c in range(col):
                rect = QGraphicsRectItem(r*self.GRID_SIZE, c*self.GRID_SIZE,
                                         self.GRID_SIZE, self.GRID_SIZE)
                rect.setBrush(QBrush(QColor(rgb2hex(self.cmap(0)))))
                self.scene.addItem(rect)
                text = self.scene.addText(f'{r} {c}')
                text.setPos(r*self.GRID_SIZE+self.GRID_SIZE/3, c*self.GRID_SIZE+self.GRID_SIZE/3)

                grids[r].append(self.Grid(rect, text))
        return grids

    @dataclass
    class Grid:
        rect: QRectF
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

        #self.mainTabs.addTab(self.mainTabWidgetPage1, 'asd')
        #self.mainTabs.addTab(self.mainTabWidgetPage1, 'asd2')
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

        self.findChild(QPushButton, 'btnSetSensor').clicked.connect(self._set)

        # Get references to children
        self.comboSensor = self.findChild(QComboBox, 'comboSensor')
        self.comboPowerMode = self.findChild(QComboBox, 'comboPowerMode')
        self.comboResolution = self.findChild(QComboBox, 'comboResolution')
        self.editI2CAddress = self.findChild(QLineEdit, 'editI2CAddress')
        self.editRangingFrequency = self.findChild(QLineEdit, 'editRangingFrequency')
        self.editIntegrationTime = self.findChild(QLineEdit, 'editIntegrationTime')
        self.editSharpener = self.findChild(QLineEdit, 'editSharpener')
        self.comboTargerOrder = self.findChild(QComboBox, 'comboTargerOrder')
        self.comboRangingMode = self.findChild(QComboBox, 'comboRangingMode')

        self._populate_comboboxes()

        self._string_to_sensor = {sensor.name: sensor.value for sensor in SlamdeckSensor}
        self._string_to_power_mode = {mode.name: mode.value for mode in VL53L5CX_PowerMode}
        self._string_to_resolution = {res.name: res.value for res in VL53L5CX_Resolution}
        self._string_to_target_order = {target.name: target.value for target in VL53L5CX_TargetOrder}
        self._string_to_ranging_mode = {mode.name: mode.value for mode in VL53L5CX_RangingMode}

    def _current_sensor(self) -> SlamdeckSensor:
        return self._string_to_sensor[self.comboSensor.currentText()]

    def _get(self) -> None:
        sensor = self._current_sensor()
        self.slamdeck.get_sensor(sensor).subscribe_to_all(self)
        #self.slamdeck.get_i2c_address()

    def on_change_all(self, attr: str, value: object):
        print('------------ ON CHANGE -------------')
        print(attr, value)


    def _set(self) -> None:
        sensor_id = self._string_to_sensor[self.comboSensor.currentText()]
        power_mode = self.comboPowerMode.currentText()
        resolution = self.comboResolution.currentText()
        target_order = self.comboTargerOrder.currentText()
        ranging_mode = self.comboRangingMode.currentText()
        i2c_address = self.editI2CAddress.text()
        ranging_frequency = self.editRangingFrequency.text()
        integration_time = self.editIntegrationTime.text()
        sharpener = self.editSharpener.text()
        print(
            f'sensor_id: {sensor_id}\n'
            f'power_mode: {power_mode}\n'
            f'resolution: {resolution}\n'
            f'target_order: {target_order}\n'
            f'ranging_mode: {ranging_mode}\n'
            f'i2c_address: {i2c_address}\n'
            f'ranging_frequency: {ranging_frequency}\n'
            f'integration_time: {integration_time}\n'
            f'sharpener: {sharpener}\n'
        )
        self._get()

    def _populate_comboboxes(self) -> None:
        self.comboSensor.addItems([sensor.name for sensor in SlamdeckSensor])
        self.comboPowerMode.addItems([mode.name for mode in VL53L5CX_PowerMode])
        self.comboResolution.addItems([res.name for res in VL53L5CX_Resolution])
        self.comboTargerOrder.addItems([target.name for target in VL53L5CX_TargetOrder])
        self.comboRangingMode.addItems([mode.name for mode in VL53L5CX_RangingMode])
        """
        self.editI2CAddress
        self.editRangingFrequency
        self.editIntegrationTime
        self.editSharpener
        """


    def _get_sensor_data_thread(self) -> None:
        global received
        update_frequency_ms = 1000
        sleep = update_frequency_ms * (1/1000)
        while self._sampling.is_set() and self.slamdeck.is_connected():
            if received:
                self.slamdeck.get_data_from_sensor(SlamdeckSensor.BACK)
                received = False

    def _start_sampling(self) -> None:
        self._sampling.set()
        Thread(target=self._get_sensor_data_thread, daemon=True).start()

    def _test(self) -> None:
        global t0
        t0 = time.time()
        self.slamdeck.get_data_from_sensor(SlamdeckSensor.BACK)

    def _connect(self) -> None:
        if not self.slamdeck.is_connected():
            self.slamdeck.connect()

    def _disconnect(self) -> None:
        if self.slamdeck.is_connected():
            self._sampling.clear()
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
