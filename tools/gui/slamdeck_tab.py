from dataclasses import dataclass
import logging
from enum import IntEnum
import typing as t

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QPointF, QSizeF, QRectF, QObject, QTimer, QThread
from PyQt5.QtWidgets import (QMessageBox, QWidget, QGraphicsScene, QGraphicsView,
                            QGraphicsTextItem, QGraphicsRectItem, QComboBox,
                            QLabel, QLineEdit, QPushButton, QPlainTextEdit,
                            QTabWidget, QOpenGLWidget, QFrame, QVBoxLayout)
from PyQt5.QtGui import QPen, QColor, qRgb, QTextBlock, QBrush, QImage
from pathlib import Path

from matplotlib.colors import LinearSegmentedColormap, rgb2hex

from cflib.crazyflie import Crazyflie


# Gui imports
from tab import Tab
from visualizer_3d import Visualizer3d
from flight_controller import FlightController
from slamdeck_python.transport_ip import TransportIp
from slamdeck_python.cpx import BackendCPX
from slamdeck_python.slamdeck import (Slamdeck, SlamdeckSensorId, SlamdeckSettings,
                                      VL53L5CX, VL53L5CX_PowerMode, VL53L5CX_RangingMode,
                                      VL53L5CX_Resolution, VL53L5CX_TargetOrder)
from slamdeck_python.utils import Callback
from slamdeck_python.cpx_with_crtp import BackendCPXWithCrtp


logger = logging.getLogger(__name__)

slamdeck_class = uic.loadUiType(Path(__file__).parent.joinpath('ui/slamdeck_tab.ui'))[0]


class SensorDimension(IntEnum):
    DIMENSION_4X4 = 4
    DIMENSION_8X8 = 8

N = 2000
cmap = LinearSegmentedColormap.from_list('r',["r", "w", "g"], N=N)


class SensorUi(QObject):

    _s_update_ui = pyqtSignal()
    _s_create_grids = pyqtSignal()
    FPS = 30

    colors = [QBrush(QColor(rgb2hex(cmap(value)))) for value in range(N+1)]

    def __init__(self, graphics: QGraphicsView, sensor: VL53L5CX, small: bool = False):
        QObject.__init__(self)
        self.graphics = graphics
        self.sensor = sensor

        # TODO: Check if this makes difference
        self.graphics.setInteractive(False)
        self.scene = QGraphicsScene()
        self.graphics.setScene(self.scene)
        self._is_small = small

        self._s_create_grids.connect(self._create_grids)

        # Add subscriptions
        self.grid_per_row: int = 0
        self.set_sensor(sensor)

        self._graph_timer = QTimer()
        self._graph_timer.setInterval(int(1000 / self.FPS))
        self._graph_timer.timeout.connect(self._update_graphics)
        self._graph_timer.start()

    def set_sensor(self, sensor: VL53L5CX):
        self.sensor = sensor
        if self.sensor.resolution == VL53L5CX_Resolution.RESOLUTION_4X4:
            self.grid_per_row = 4
        else:
            self.grid_per_row = 8
        self._s_create_grids.emit()

    def _constraint_value(self, value) -> int:
        if value > N:
            value = N
        return value

    def _update_graphics(self) -> None:
        g = 0
        distances = self.sensor.data

        for r in range(self.grid_per_row):
            for c in range(self.grid_per_row):
                grid = self.grids[self.grid_per_row-1-r][c]
                value = distances[g]
                color_value = self._constraint_value(value)
                grid.rect.setBrush(self.colors[color_value])
                g += 1

    def _create_grids(self) -> t.List[t.List['Grid']]:
        self.scene.clear()
        self.grids: t.List[t.List['Grid']] = []
        if self.grid_per_row == SensorDimension.DIMENSION_4X4:
            grid_size = 150
        else:
            grid_size = 70

        if self._is_small:
            grid_size *= 0.3

        for r in range(self.grid_per_row):
            self.grids.append([])
            for c in range(self.grid_per_row):
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


class SlamdeckTab(Tab, slamdeck_class):

    _s_connecting = pyqtSignal()
    _s_connected = pyqtSignal()
    _s_disconnected = pyqtSignal()
    _s_connection_error = pyqtSignal(str)
    _s_on_new_data = pyqtSignal()
    _s_update_sensor_settings_ui = pyqtSignal()

    PACKET_SIZE = 640
    UPDATE_FREQUENCY = 30

    DEFAULT_IP = '192.168.6.83'
    DEFAULT_PORT = '5000'

    def __init__(self, tabWidget, helper, *args):
        super(SlamdeckTab, self).__init__(*args)
        self.setupUi(self)
        self.tabWidget = tabWidget

        self._cf = Crazyflie(rw_cache='./cache')

        # Attach button callbacks
        self.btnConnect.clicked.connect(self._connect)
        self.btnDisconnect.clicked.connect(self._disconnect)
        self.btnTest.clicked.connect(self._test)
        self.btnStartStreaming.clicked.connect(self._start_streaming)
        self.btnStopStreaming.clicked.connect(self._stop_streaming)

        self.findChild(QPushButton, 'btnSetSensor').clicked.connect(self._set_settings)

        # Attach PyQt signals to callback functions
        self._s_on_new_data.connect(self._cb_on_new_data)
        self._s_connecting.connect(self._cb_connecting)
        self._s_connected.connect(self._cb_connected)
        self._s_disconnected.connect(self._cb_disconnected)
        self._s_connection_error.connect(self._cb_connection_error)

        self._s_update_sensor_settings_ui.connect(self._cb_update_sensor_settings_ui)

        # Get references to children
        self.comboSensor: QComboBox = self.findChild(QComboBox, 'comboSensor')
        self.comboPowerMode: QComboBox = self.findChild(QComboBox, 'comboPowerMode')
        self.comboResolution: QComboBox = self.findChild(QComboBox, 'comboResolution')
        self.editRangingFrequency: QLineEdit = self.findChild(QLineEdit, 'editRangingFrequency')
        self.editIntegrationTime: QLineEdit = self.findChild(QLineEdit, 'editIntegrationTime')
        self.editSharpener: QLineEdit = self.findChild(QLineEdit, 'editSharpener')
        self.comboTargerOrder: QComboBox = self.findChild(QComboBox, 'comboTargerOrder')
        self.comboRangingMode: QComboBox = self.findChild(QComboBox, 'comboRangingMode')


        self.mainTabs: QTabWidget = self.findChild(QTabWidget, 'mainTabs')
        self.graphics2dVisualizer: QGraphicsView = self.findChild(QGraphicsView, 'tabSensorGraphics')
        self.graphics3dVisualizer: QFrame = self.findChild(QFrame, 'graphics3dVisualizer')

        self.editPort = self.findChild(QLineEdit, 'editPort')

        self._string_to_sensor = {sensor.name: sensor.value for sensor in SlamdeckSensorId}
        self._string_to_power_mode = {mode.name: mode.value for mode in VL53L5CX_PowerMode}
        self._string_to_resolution = {res.name: res.value for res in VL53L5CX_Resolution}
        self._string_to_target_order = {target.name: target.value for target in VL53L5CX_TargetOrder}
        self._string_to_ranging_mode = {mode.name: mode.value for mode in VL53L5CX_RangingMode}

        self._populate_comboboxes()
        self._set_default_values()

        #self.slamdeck = Slamdeck(BackendCPX(TransportIp(self._get_ip(), self._get_port())))
        self.slamdeck = Slamdeck(BackendCPXWithCrtp(self._get_ip(), self._get_port(), self._cf), self)

        # Attach callbacks for Slamdeck to trigger PyQt signals
        self.slamdeck.connecting_handler.add_callback(Callback(self._s_connecting.emit))
        self.slamdeck.connected_handler.add_callback(Callback(self._s_connected.emit))
        self.slamdeck.connection_error_handler.add_callback(Callback(self._s_connection_error.emit))
        self.slamdeck.disconnected_handler.add_callback(Callback(self._s_disconnected.emit))
        self.slamdeck.on_new_data_handler.add_callback(Callback(self._s_on_new_data.emit))

        # Build sensor UI graphics.
        self.sensor_single = SensorUi(self.singleSensor, self.slamdeck.get_sensor(SlamdeckSensorId.BACK))
        self.sensor_front = SensorUi(self.sensorFront, self.slamdeck.get_sensor(SlamdeckSensorId.FRONT), small=True)
        self.sensor_main = SensorUi(self.sensorMain, self.slamdeck.get_sensor(SlamdeckSensorId.MAIN), small=True)
        self.sensor_right = SensorUi(self.sensorRight, self.slamdeck.get_sensor(SlamdeckSensorId.RIGHT), small=True)
        self.sensor_back = SensorUi(self.sensorBack, self.slamdeck.get_sensor(SlamdeckSensorId.BACK), small=True)
        self.sensor_left = SensorUi(self.sensorLeft, self.slamdeck.get_sensor(SlamdeckSensorId.LEFT), small=True)

        self._update_button_states(connected=False)

        # Create 3d visualizer, which must be done after slamdeck instantiation
        self.visualizer_3d = Visualizer3d(self.slamdeck)
        lay = QVBoxLayout(self.graphics3dVisualizer)
        lay.addWidget(self.visualizer_3d.native)

        self._fc = FlightController(self._cf, self.visualizer_3d)
        self.vbat: float = 0.0

        self._graph_timer = QTimer()
        self._graph_timer.setInterval(int(1000 / self.UPDATE_FREQUENCY))
        self._graph_timer.timeout.connect(self._update_ui)
        self._graph_timer.start()

        self.comboSensor.currentTextChanged.connect(self._sensor_changed)
        self._s_update_sensor_settings_ui.emit()

    def _sensor_changed(self) -> None:
        self.sensor_single.set_sensor(self._current_sensor())

    def _current_sensor(self) -> SlamdeckSensorId:
        sensor_id = SlamdeckSensorId(self._string_to_sensor[self.comboSensor.currentText()])
        return self.slamdeck.get_sensor(sensor_id)

    def _get_ip(self) -> str:
        return self.editIp.text()

    def _get_port(self) -> int:
        return int(self.editPort.text())

    def _set_default_values(self) -> None:
        self.editIp.setText(self.DEFAULT_IP)
        self.editPort.setText(self.DEFAULT_PORT)

    def _set_settings(self) -> None:
        settings = SlamdeckSettings(
            int(self.editIntegrationTime.text()),
            int(self.editSharpener.text()),
            int(self.editRangingFrequency.text()),
            VL53L5CX_Resolution(self._string_to_resolution[self.comboResolution.currentText()]),
            VL53L5CX_PowerMode(self._string_to_power_mode[self.comboPowerMode.currentText()]),
            VL53L5CX_TargetOrder(self._string_to_target_order[self.comboTargerOrder.currentText()]),
            VL53L5CX_RangingMode(self._string_to_ranging_mode[self.comboRangingMode.currentText()])
        )
        self.slamdeck.set_settings(settings)

    def _populate_comboboxes(self) -> None:
        self.comboSensor.addItems(self._string_to_sensor.keys())
        self.comboPowerMode.addItems(self._string_to_power_mode.keys())
        self.comboResolution.addItems(self._string_to_resolution.keys())
        self.comboTargerOrder.addItems(self._string_to_target_order.keys())
        self.comboRangingMode.addItems(self._string_to_ranging_mode.keys())

    def _start_streaming(self) -> None:
        self.slamdeck.start_streaming()

    def _stop_streaming(self) -> None:
        self.slamdeck.stop_streaming()

    def _test(self) -> None:
        self.slamdeck.get_data()

    def _connect(self) -> None:
        self.i = 0
        self.t0 = 0

        # TODO: Fix his private attribute access
        if not self.slamdeck.is_connected():
            self.slamdeck._backend._transport._ip = self._get_ip()
            self.slamdeck._backend._transport._port = self._get_port()
            self.slamdeck.connect()

    def _disconnect(self) -> None:
        self.slamdeck.disconnect()

    def _update_ui(self) -> None:
        self.labelSampleRate.setText(str(self.slamdeck.get_streaming_rate()))
        self.labelBattery.setText(str(round(self.vbat, 3)))

    def _update_button_states(self, connected: bool) -> None:
        self.btnDisconnect.setEnabled(connected)
        self.btnConnect.setEnabled(not connected)
        self.btnTest.setEnabled(connected)
        self.btnStartStreaming.setEnabled(connected)
        self.btnStopStreaming.setEnabled(connected)

    # --- Callbacks -- #
    def _cb_on_new_data(self) -> None:
        pass

    def _cb_connecting(self) -> None:
        logger.debug('Connecting...')
        self.statusText.setText('Connecting')

    def _cb_connected(self) -> None:
        logger.debug('Connected')
        self.statusText.setText('Connected')
        self._update_button_states(True)
        self._s_update_sensor_settings_ui.emit()

    def _cb_disconnected(self) -> None:
        logger.debug('Disconnected')
        self.statusText.setText('Disconnected')
        self._update_button_states(False)

    def _cb_update_sensor_settings_ui(self) -> None:
        sensor = self._current_sensor()
        self.editRangingFrequency.setText(str(sensor.ranging_frequency_hz))
        self.editIntegrationTime.setText(str(sensor.integration_time_ms))
        self.editSharpener.setText(str(sensor.sharpener_percent))
        self.comboPowerMode.setCurrentText(sensor.power_mode.name)
        self.comboResolution.setCurrentText(sensor.resolution.name)
        self.comboTargerOrder.setCurrentText(sensor.target_order.name)
        self.comboRangingMode.setCurrentText(sensor.ranging_mode.name)

    def _cb_connection_error(self, msg: str):
        logger.debug(f'Error: {msg}')
        self.statusText.setText(msg)

    def _logging_error(self, log_conf, msg):
        """Callback from the log layer when an error occurs"""

        QMessageBox.about(self, "Example error",
                          "Error when using log config"
                          " [{0}]: {1}".format(log_conf.name, msg))
