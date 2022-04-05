from dataclasses import dataclass
import logging
from threading import Thread, Event, currentThread

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QPointF, QSizeF, QRectF, QObject
from PyQt5.QtWidgets import (QMessageBox, QWidget, QGraphicsScene, QGraphicsView,
                            QGraphicsTextItem, QGraphicsRectItem, QComboBox,
                            QLabel, QLineEdit, QPushButton, QPlainTextEdit,
                            QTabWidget, QOpenGLWidget, QFrame, QVBoxLayout)
from PyQt5 import Qt
from PyQt5.QtGui import QPen, QColor, qRgb, QTextBlock, QBrush, QImage
from pathlib import Path

from PIL import Image
from PIL.ImageQt import ImageQt

import numpy as np
from  matplotlib.colors import LinearSegmentedColormap, rgb2hex


from tab import Tab
from slamdeck_python.slamdeck import Slamdeck
from slamdeck_python.backend_cpx import BackendCPX, BackendCPXProtocol
from slamdeck_python.backend import BackendParams, Callback
from slamdeck_python.backend_ip import BackendIp
from slamdeck_python.slamdeck import Sensor
from slamdeck_python.slamdeck_api import SlamdeckSensor, VL53L5CX_PowerMode, VL53L5CX_RangingMode, VL53L5CX_Resolution, VL53L5CX_TargetOrder
from slamdeck_python.utils import Subscriber, subscriber
from vispy_tab import VispyTest


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

N = 500
cmap = LinearSegmentedColormap.from_list('rg',["r", "w", "g"], N=N)


class SensorUi(QObject, Subscriber):

    _s_update_ui = pyqtSignal()
    _s_create_grids = pyqtSignal()

    colors = [QBrush(QColor(rgb2hex(cmap(value)))) for value in range(N+1)]

    def __init__(self, graphics: QGraphicsView, sensor: Sensor, small: bool = False):
        QObject.__init__(self)
        self.graphics = graphics
        self.sensor = sensor

        # TODO: Check if this makes difference
        self.graphics.setInteractive(False)
        self.scene = QGraphicsScene()
        self.graphics.setScene(self.scene)
        self.dimension = 4
        self._is_small = small

        self._s_update_ui.connect(self._cb_update_ui)
        self._s_create_grids.connect(self._create_grids)

        # Add subscriptions
        self.set_sensor(sensor)

    def set_sensor(self, sensor: Sensor):
        #self.sensor.unsubscribe(self)
        self.sensor = sensor
        self.sensor.subscribe(self, 'data')
        self.set_dimension(self.sensor.resolution)
        self._s_create_grids.emit()

    def set_dimension(self, resolution: VL53L5CX_Resolution) -> None:
        if resolution == VL53L5CX_Resolution.RESOLUTION_4X4:
            self.dimension = 4
        else:
            self.dimension = 8

    def on_change(self, value: np.ndarray) -> None:
        self._s_update_ui.emit()

    def _cb_update_ui(self) -> None:
        #rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        #PIL_image = Image.fromarray(rgb_image).convert('RGB')
        #return QPixmap.fromImage(ImageQt(PIL_image))
        g = 0
        distances = self.sensor.data

        for r in range(self.dimension):
            for c in range(self.dimension):
                grid = self.grids[self.dimension-1-r][c]
                value = distances[g]
                color_value = N if value >= N else value
                grid.rect.setBrush(self.colors[color_value])
                #grid.text.setPlainText(str(value))
                g += 1

    def _create_grids(self) -> t.List[t.List['Grid']]:
        self.scene.clear()
        self.grids: t.List[t.List['Grid']] = []
        if self.dimension == SensorDimension.DIMENSION_4X4:
            grid_size = 150
        else:
            grid_size = 70

        if self._is_small:
            grid_size *= 0.3

        for r in range(self.dimension):
            self.grids.append([])
            for c in range(self.dimension):
                rect = QGraphicsRectItem(c*grid_size, r*grid_size,
                                         grid_size, grid_size)
                rect.setBrush(self.colors[0])
                self.scene.addItem(rect)
                #text = self.scene.addText(f'{r} {c}')
                text = self.scene.addText(f'')
                text.setPos(c*grid_size+grid_size/3, r*grid_size+grid_size/3)

                self.grids[r].append(self.Grid(rect, text))

        return self.grids

    @dataclass
    class Grid:
        rect: QRectF
        text: QGraphicsTextItem


class ContinousSamplingThread(Thread):

    def __init__(self, slamdeck: Slamdeck, sensor: SlamdeckSensor):
        super().__init__(daemon=True)
        self._slamdeck = slamdeck
        self._sensor = sensor
        self._is_running = Event()
        self._has_received_response = Event()

    def received_response(self) -> None:
        """ Should be called from different thread to indicate that we've received
            a response from the last data request.
        """
        self._has_received_response.set()

    def stop(self) -> None:
        self._is_running.clear()
        self.received_response()

    def _received_response(self) -> bool:
        return self._has_received_response.is_set()

    def _clear_response(self) -> bool:
        return self._has_received_response.clear()

    def run(self) -> None:
        self._is_running.set()
        logger.debug(f'Sampling thread started: {currentThread().getName()}')

        while self._is_running.is_set() and self._slamdeck.is_connected():
            self._slamdeck.get_data_from_sensor(self._sensor)

            while not self._received_response():
                pass

            self._clear_response()

        logger.debug('Sampling thread ended.')


class SlamdeckTab(Tab, example_tab_class):

    _s_connecting = pyqtSignal()
    _s_connected = pyqtSignal()
    _s_disconnected = pyqtSignal()
    _s_connection_error = pyqtSignal(str)
    _s_on_new_data = pyqtSignal()
    _s_update_sensor_config_ui = pyqtSignal(Sensor)

    PACKET_SIZE = 640

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

        self._s_update_sensor_config_ui.connect(self._cb_update_sensor_config_ui)

        self.findChild(QPushButton, 'btnSetSensor').clicked.connect(self._set)
        self.findChild(QPushButton, 'btnUpdateConfig').clicked.connect(self._get_sensor_config)

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
        #self.graphics2dVisualizer: QGraphicsView = self.findChild(QGraphicsView, 'tabSensorGraphics')
        #self.graphics3dVisualizer: QOpenGLWidget = self.findChild(QOpenGLWidget, 'graphics3dVisualizer')
        #self.v = VispyTest()
        #self.graphics3dVisualizer: QFrame = self.findChild(QFrame, 'graphics3dVisualizer')
        #lay = QVBoxLayout(self.graphics3dVisualizer)
        #lay.addWidget(self.v.native)
        #self.v.show()

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
        self.sensor_single = SensorUi(self.singleSensor, self.slamdeck.get_sensor_model(SlamdeckSensor.BACK))
        self.sensor_front = SensorUi(self.sensorFront, self.slamdeck.get_sensor_model(SlamdeckSensor.FRONT), small=True)
        self.sensor_main = SensorUi(self.sensorMain, self.slamdeck.get_sensor_model(SlamdeckSensor.MAIN), small=True)
        self.sensor_right = SensorUi(self.sensorRight, self.slamdeck.get_sensor_model(SlamdeckSensor.RIGHT), small=True)
        self.sensor_back = SensorUi(self.sensorBack, self.slamdeck.get_sensor_model(SlamdeckSensor.BACK), small=True)
        self.sensor_left = SensorUi(self.sensorLeft, self.slamdeck.get_sensor_model(SlamdeckSensor.LEFT), small=True)

        self._update_button_states(connected=False)


        config_sensor = self._current_sensor()
        if config_sensor == SlamdeckSensor.ALL:
            config_sensor = self.slamdeck.SENORS[0]
        self._cb_update_sensor_config_ui(self.slamdeck.get_sensor_model(config_sensor))

        # Create sampling thread reference
        self._sampling_thread: ContinousSamplingThread = None

    def _current_sensor(self) -> SlamdeckSensor:
        try:
            sensor_tab = self._get_active_sensor_tab()
            if sensor_tab == 'tabSingleSensor':
                sensor = SlamdeckSensor(self._string_to_sensor[self.comboSensor.currentText()])
            elif sensor_tab == 'tabAllSensor':
                sensor = SlamdeckSensor.ALL
            return sensor
        except Exception as e:
            logging.error(f'Error getting current sensor: {e}')
            return None

    def _get_ip(self) -> str:
        return self.editIp.text()

    def _get_port(self) -> int:
        return int(self.editPort.text())

    def _set_default_values(self) -> None:
        self.editIp.setText('192.168.6.83')
        self.editPort.setText('5000')

    def _get_sensor_config(self) -> None:
        config_sensor = self._current_sensor()
        if config_sensor == SlamdeckSensor.ALL:
            # If ALL sensors is set, just get the first one for the configs (all will have same configs.)
            config_sensor = self.slamdeck.SENORS[0]

        sensor_model = self.slamdeck.get_initial_sensor_settings(config_sensor)
        sensor_model.subscribe_to_all(self)

    def _cb_update_sensor_config_ui(self, sensor_model: Sensor) -> None:
        self.editRangingFrequency.setText(str(sensor_model.ranging_frequency_hz))
        self.editIntegrationTime.setText(str(sensor_model.integration_time_ms))
        self.editSharpener.setText(str(sensor_model.sharpener_percent))
        self.comboPowerMode.setCurrentText(sensor_model.power_mode.name)
        self.comboResolution.setCurrentText(sensor_model.resolution.name)
        self.comboTargerOrder.setCurrentText(sensor_model.target_order.name)
        self.comboRangingMode.setCurrentText(sensor_model.ranging_mode.name)

    def on_change_all(self, attr: str, value: object):
        print('------------ ON CHANGE -------------')

        if attr == 'resolution':
            self._update_sensor_plot()

        """
        sensors = []
        sensor = self._current_sensor()
        if sensor == SlamdeckSensor.ALL:
            sensors = self.slamdeck.SENORS
        else:
            sensors.append(sensor)

        for sensor in sensors:
            sensor_model = self.slamdeck.get_sensor_model(sensor)
            """

    def _set(self) -> None:
        sensor = SlamdeckSensor(self._string_to_sensor[self.comboSensor.currentText()])
        power_mode = VL53L5CX_PowerMode(self._string_to_power_mode[self.comboPowerMode.currentText()])
        resolution = VL53L5CX_Resolution(self._string_to_resolution[self.comboResolution.currentText()])
        target_order = self._string_to_target_order[self.comboTargerOrder.currentText()]
        ranging_mode = self._string_to_ranging_mode[self.comboRangingMode.currentText()]
        ranging_frequency = int(self.editRangingFrequency.text())
        integration_time = int(self.editIntegrationTime.text())
        sharpener = int(self.editSharpener.text())
        """
        print(
            f'sensor_id: {sensor}\n'
            f'power_mode: {power_mode}\n'
            f'resolution: {resolution}\n'
            f'target_order: {target_order}\n'
            f'ranging_mode: {ranging_mode}\n'
            f'ranging_frequency: {ranging_frequency}\n'
            f'integration_time: {integration_time}\n'
            f'sharpener: {sharpener}\n'
        )
        """
        self.slamdeck.get_sensor_model(sensor).subscribe_to_all(self)
        #self.slamdeck.set_power_mode(sensor, power_mode)
        self.slamdeck.set_resolution(sensor, resolution)
        self.slamdeck.set_ranging_frequency_hz(sensor, ranging_frequency)
        #self.slamdeck.set_target_order(sensor, target_order)
        #self.slamdeck.set_ranging_mode(sensor, ranging_mode)
        #self.slamdeck.set_integration_time_ms(sensor, integration_time)
        #self.slamdeck.set_sharpener_percent(sensor, sharpener)
        #self.slamdeck.set_resolution(sensor, resolution)

    def _populate_comboboxes(self) -> None:
        self.comboSensor.addItems(self._string_to_sensor.keys())
        self.comboPowerMode.addItems(self._string_to_power_mode.keys())
        self.comboResolution.addItems(self._string_to_resolution.keys())
        self.comboTargerOrder.addItems(self._string_to_target_order.keys())
        self.comboRangingMode.addItems(self._string_to_ranging_mode.keys())

    def _get_active_sensor_tab(self) -> str:
        return self.tabSensorGraphics.currentWidget().objectName()

    def _start_sampling(self) -> None:
        if self._sampling_thread is not None:
            logger.warning('Already running continous sampling')
            return

        #self._update_sensor_plot()
        self._sampling_thread = ContinousSamplingThread(self.slamdeck, self._current_sensor())
        self._sampling_thread.start()

    def _update_sensor_plot(self) -> None:
        sensor = self._current_sensor()
        sensor_model = self.slamdeck.get_sensor_model(sensor)
        sensor_model.subscribe_to_all(self)

    def _test(self) -> None:
        sensor = self._current_sensor()
        self.slamdeck.get_data_from_sensor(sensor)
        #self.sensor_single.set_sensor(self.slamdeck.get_sensor_model(sensor))
        #self.slamdeck.get_data_from_sensor(sensor)

    def _connect(self) -> None:
        self.i = 0
        self.t0 = 0

        # TODO: Fix his private attribute access
        self.slamdeck._backend._ip = self._get_ip()
        self.slamdeck._backend._port = self._get_port()
        if not self.slamdeck.is_connected():
            self.slamdeck.connect()

    def _disconnect(self) -> None:
        self.slamdeck.disconnect()

        if self._sampling_thread is not None:
            self._sampling_thread.stop()
            self._sampling_thread = None

    # --- Callbacks -- #
    def _cb_on_new_data(self) -> None:
        # Notify samping thread that we've received the response
        if self._sampling_thread is not None:
            self._sampling_thread.received_response()
            now = time.time()
            if self.t0 == 0:
                self.t0 = now

            self.i += 1
            dt = now - self.t0
            if dt > 1:
                print(f'{self.i} Hz, {self.i*self.PACKET_SIZE} bytes')
                self.i = 0
                self.t0 = now


    def _cb_connecting(self) -> None:
        logger.debug('Connecting...')
        self.statusText.setText('Connecting')

    def _cb_connected(self) -> None:
        logger.debug('Connected')
        self.statusText.setText('Connected')
        self._update_button_states(True)

    def _cb_disconnected(self) -> None:
        logger.debug('Disconnected')
        self.statusText.setText('Disconnected')
        self._update_button_states(False)

    def _update_button_states(self, connected: bool) -> None:
        self.btnDisconnect.setEnabled(connected)
        self.btnConnect.setEnabled(not connected)
        self.btnTest.setEnabled(connected)
        self.btnStartSampling.setEnabled(connected)

    def _cb_connection_error(self, msg: str):
        logger.debug(f'Error: {msg}')
        self.statusText.setText(msg)

    def _logging_error(self, log_conf, msg):
        """Callback from the log layer when an error occurs"""

        QMessageBox.about(self, "Example error",
                          "Error when using log config"
                          " [{0}]: {1}".format(log_conf.name, msg))
