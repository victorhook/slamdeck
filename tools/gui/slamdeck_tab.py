from dataclasses import dataclass
import logging
from threading import Thread, Event, currentThread

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QPointF, QSizeF, QRectF, QObject
from PyQt5.QtWidgets import (QMessageBox, QWidget, QGraphicsScene, QGraphicsView,
                            QGraphicsTextItem, QGraphicsRectItem, QComboBox,
                            QLabel, QLineEdit, QPushButton, QPlainTextEdit,
                            QTabWidget)
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

class SensorDimension(IntEnum):
    DIMENSION_4X4 = 4
    DIMENSION_8X8 = 8


class SensorUi(QObject, Subscriber):

    _s_distance = pyqtSignal(np.ndarray)
    cmap = LinearSegmentedColormap.from_list('rg',["r", "w", "g"], N=500)

    def __init__(self, graphics: QGraphicsView, sensor: Sensor):
        QObject.__init__(self)
        self.graphics = graphics
        self.sensor = sensor

        self.dimesion: SensorDimension = SensorDimension.DIMENSION_4X4
        self.scene = QGraphicsScene()
        self.graphics.setScene(self.scene)
        self.grids = self._create_grids()

        self._s_distance.connect(self._cb_distance)

        # Add subscriptions
        self.sensor.subscribe(self, 'data')

    def set_dimension(self, dimension: SensorDimension) -> None:
        if dimension == self.dimesion:
            return

        self.dimesion = dimension
        self.scene.clear()
        self.grids = self._create_grids()


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

    def _create_grids(self) -> t.List[t.List['Grid']]:
        grids: t.List[t.List['Grid']] = []
        grid_size = int(self.graphics.width() / self.dimesion)
        grid_size *= 0.95

        for r in range(self.dimesion):
            grids.append([])
            for c in range(self.dimesion):
                rect = QGraphicsRectItem(r*grid_size, c*grid_size,
                                         grid_size, grid_size)
                rect.setBrush(QBrush(QColor(rgb2hex(self.cmap(0)))))
                self.scene.addItem(rect)
                text = self.scene.addText(f'{r} {c}')
                text.setPos(r*grid_size+grid_size/3, c*grid_size+grid_size/3)

                grids[r].append(self.Grid(rect, text))

        return grids

    @dataclass
    class Grid:
        rect: QRectF
        text: QGraphicsTextItem


class ContinousSamplingThread(Thread):

    def __init__(self, slamdeck: Slamdeck, max_sampling_frequency_hz: int = 60):
        super().__init__(daemon=True)
        self._slamdeck = slamdeck
        self._max_sampling_frequency_hz = max_sampling_frequency_hz
        self._max_delay = max_sampling_frequency_hz * (1/1000)
        self._is_running = Event()
        self._has_received_response = Event()

    def received_response(self) -> None:
        """ Should be called from different thread to indicate that we've received
            a response from the last data request.
        """
        self._has_received_response.set()

    def stop(self) -> None:
        self._is_running.clear()

    def _received_response(self) -> bool:
        return self._has_received_response.is_set()

    def run(self) -> None:
        self._is_running.set()
        logging.debug(f'Sampling thread started: {currentThread().getName()}')

        t0 = time.time()
        while self._is_running.is_set() and self._slamdeck.is_connected():

            while not self._received_response():
                pass

            t1 = time.time()
            dt = t1 - t0

            if (dt > self._max_delay) and received:
                self.slamdeck.get_data_from_sensor(SlamdeckSensor.BACK)
                t0 = t1

        logging.debug('Sampling thread ended.')


class SlamdeckTab(Tab, example_tab_class):

    _s_connecting = pyqtSignal()
    _s_connected = pyqtSignal()
    _s_disconnected = pyqtSignal()
    _s_connection_error = pyqtSignal(str)
    _s_on_new_data = pyqtSignal()

    def __init__(self, tabWidget, helper, *args):
        super(SlamdeckTab, self).__init__(*args)
        self.setupUi(self)
        self.tabWidget = tabWidget

        # Attach button callbacks
        self.btnConnect.clicked.connect(self._connect)
        self.btnDisconnect.clicked.connect(self._disconnect)
        self.btnTest.clicked.connect(self._test)
        self.btnStartSampling.clicked.connect(self._start_sampling)

        # Attach PyQt signals to callback functions
        self._s_on_new_data.connect(self._cb_on_new_data)
        self._s_connecting.connect(self._cb_connecting)
        self._s_connected.connect(self._cb_connected)
        self._s_disconnected.connect(self._cb_disconnected)
        self._s_connection_error.connect(self._cb_connection_error)

        self.findChild(QPushButton, 'btnSetSensor').clicked.connect(self._set)

        # Get references to children
        self.debugRaw = self.findChild(QPlainTextEdit, 'debugRaw')
        self.debugCPX = self.findChild(QPlainTextEdit, 'debugCPX')

        self.comboSensor: QComboBox = self.findChild(QComboBox, 'comboSensor')
        self.comboPowerMode: QComboBox = self.findChild(QComboBox, 'comboPowerMode')
        self.comboResolution: QComboBox = self.findChild(QComboBox, 'comboResolution')
        self.editRangingFrequency: QLineEdit = self.findChild(QLineEdit, 'editRangingFrequency')
        self.editIntegrationTime: QLineEdit = self.findChild(QLineEdit, 'editIntegrationTime')
        self.editSharpener: QLineEdit = self.findChild(QLineEdit, 'editSharpener')
        self.comboTargerOrder: QComboBox = self.findChild(QComboBox, 'comboTargerOrder')
        self.comboRangingMode: QComboBox = self.findChild(QComboBox, 'comboRangingMode')

        self.mainTabs: QTabWidget = self.findChild(QTabWidget, 'mainTabs')
        self.tabSensorGraphics: QTabWidget = self.findChild(QTabWidget, 'tabSensorGraphics')
        self.graphics2dVisualizer: QGraphicsView = self.findChild(QGraphicsView, 'tabSensorGraphics')

        self.editIp = self.findChild(QLineEdit, 'editIp')
        self.editPort = self.findChild(QLineEdit, 'editPort')

        self._string_to_sensor = {sensor.name: sensor.value for sensor in SlamdeckSensor}
        self._string_to_power_mode = {mode.name: mode.value for mode in VL53L5CX_PowerMode}
        self._string_to_resolution = {res.name: res.value for res in VL53L5CX_Resolution}
        self._string_to_target_order = {target.name: target.value for target in VL53L5CX_TargetOrder}
        self._string_to_ranging_mode = {mode.name: mode.value for mode in VL53L5CX_RangingMode}

        self._populate_comboboxes()
        self._set_default_values()

        self.slamdeck = Slamdeck(BackendCPX(BackendIp(self._get_ip(), self._get_port())))

        # Attach callbacks for Slamdeck to trigger PyQt signals
        self.slamdeck.add_cb_connecting(Callback(self._s_connecting.emit))
        self.slamdeck.add_cb_connected(Callback(self._s_connected.emit))
        self.slamdeck.add_cb_connection_error(Callback(self._s_connection_error.emit))
        self.slamdeck.add_cb_disconnected(Callback(self._s_disconnected.emit))
        self.slamdeck.add_cb_on_new_data(Callback(self._s_on_new_data.emit))

        # Build sensor UI graphics.
        self.sensor_single = SensorUi(self.singleSensor, self.slamdeck.get_sensor(SlamdeckSensor.BACK))
        self.sensor_front = SensorUi(self.sensorFront, self.slamdeck.get_sensor(SlamdeckSensor.FRONT))
        self.sensor_main = SensorUi(self.sensorMain, self.slamdeck.get_sensor(SlamdeckSensor.MAIN))
        self.sensor_right = SensorUi(self.sensorRight, self.slamdeck.get_sensor(SlamdeckSensor.RIGHT))
        self.sensor_back = SensorUi(self.sensorBack, self.slamdeck.get_sensor(SlamdeckSensor.BACK))
        self.sensor_left = SensorUi(self.sensorLeft, self.slamdeck.get_sensor(SlamdeckSensor.LEFT))

        # Create sampling thread reference
        self._sampling_thread: ContinousSamplingThread = None

    def _current_sensor(self) -> SlamdeckSensor:
        return self._string_to_sensor[self.comboSensor.currentText()]

    def _get(self) -> None:
        sensor = self._current_sensor()
        self.slamdeck.get_sensor(sensor).subscribe_to_all(self)

    def on_change_all(self, attr: str, value: object):
        print('------------ ON CHANGE -------------')
        print(attr, value)

    def _get_ip(self) -> str:
        return self.editIp.text()

    def _get_port(self) -> int:
        return int(self.editPort.text())

    def _set_default_values(self) -> None:
        self.editIp.setText('192.168.1.73')
        self.editPort.setText('5000')

    def _set(self) -> None:
        sensor_id = self._string_to_sensor[self.comboSensor.currentText()]
        power_mode = self.comboPowerMode.currentText()
        resolution = self.comboResolution.currentText()
        target_order = self.comboTargerOrder.currentText()
        ranging_mode = self.comboRangingMode.currentText()
        ranging_frequency = self.editRangingFrequency.text()
        integration_time = self.editIntegrationTime.text()
        sharpener = self.editSharpener.text()
        print(
            f'sensor_id: {sensor_id}\n'
            f'power_mode: {power_mode}\n'
            f'resolution: {resolution}\n'
            f'target_order: {target_order}\n'
            f'ranging_mode: {ranging_mode}\n'
            f'ranging_frequency: {ranging_frequency}\n'
            f'integration_time: {integration_time}\n'
            f'sharpener: {sharpener}\n'
        )
        self._get()

    def _populate_comboboxes(self) -> None:
        self.comboSensor.addItems(self._string_to_sensor.keys())
        self.comboPowerMode.addItems(self._string_to_power_mode.keys())
        self.comboResolution.addItems(self._string_to_resolution.keys())
        self.comboTargerOrder.addItems(self._string_to_target_order.keys())
        self.comboRangingMode.addItems(self._string_to_ranging_mode.keys())
        """
        self.editRangingFrequency
        self.editIntegrationTime
        self.editSharpener
        """

    def _get_active_sensor_tab(self) -> str:
        return self.tabSensorGraphics.currentWidget().objectName()

    def _start_sampling(self) -> None:
        if self._sampling_thread is not None:
            logging.warning('Already running continous sampling')
            return

        self._sampling_thread = ContinousSamplingThread(self.slamdeck)
        self._sampling_thread.start()

    def _test(self) -> None:
        self.slamdeck.get_data_from_sensor(SlamdeckSensor.BACK)

    def _connect(self) -> None:
        # TODO: Fix his private attribute access
        self.slamdeck._backend._ip = self._get_ip()
        self.slamdeck._backend._port = self._get_port()
        if not self.slamdeck.is_connected():
            self.slamdeck.connect()

    def _disconnect(self) -> None:
        self.slamdeck.disconnect()
        if self._sampling_thread is not None:
            self._sampling_thread.stop()

    # --- Callbacks -- #
    def _cb_on_new_data(self) -> None:
        # Notify samping thread that we've received the response
        if self._sampling_thread is not None:
            self._sampling_thread.received_response()

    def _cb_connecting(self) -> None:
        logger.debug('Connecting...')
        self.statusText.setText('Connecting')

    def _cb_connected(self) -> None:
        logger.debug('Connected')
        self.statusText.setText('Connected')

        sensor_tab = self._get_active_sensor_tab()
        if sensor_tab == 'tabSingleSensor':
            active_sensor = self.comboSensor.currentText()
        elif sensor_tab == 'tabAllSensor':
            active_sensor = SlamdeckSensor.FRONT
        else:
            logging.warning(f'Failed to recognize tab {sensor_tab}')

        #self.slamdeck.get_initial_sensor_settings(active_sensor)

    def _cb_disconnected(self) -> None:
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
