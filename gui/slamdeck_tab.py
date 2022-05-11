from dataclasses import dataclass
import logging
from enum import IntEnum
import typing as t

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QPointF, QSizeF, QRectF, QObject, QTimer, QThread
from PyQt5.QtWidgets import (QMessageBox, QWidget, QGraphicsScene, QGraphicsView,
                            QGraphicsTextItem, QGraphicsRectItem, QComboBox,
                            QLabel, QLineEdit, QPushButton, QPlainTextEdit,
                            QTabWidget, QOpenGLWidget, QFrame, QVBoxLayout, QRadioButton)

from pathlib import Path

# Gui imports
from tab import Tab
#from flight_controller import FlightController

from gui import utils
from gui.flight_controller import FlightController
from gui.slamdeck import Slamdeck, SlamdeckSensorId, SlamdeckSettings
from gui.data_link import DataLinkNrfCRTP, DataLinkSocketTCP, DataLinkType
from gui.visualizer_2d_matrix import Visualizer2dMatrix
from gui.visualizer_2d_point_cloud import Visualizer2dPointCloud
from gui.visualizer_3d import Visualizer3d
from gui.models import (VL53L5CX_PowerMode, VL53L5CX_RangingMode,
                        VL53L5CX_Resolution, VL53L5CX_Status,
                        VL53L5CX_TargetOrder,
                        ModelCrazyflie, ModelVL53L5CX)

logger = logging.getLogger(__name__)

slamdeck_class = uic.loadUiType(Path(__file__).parent.joinpath('ui/slamdeck_tab.ui'))[0]



class SlamdeckTab(Tab, slamdeck_class):

    _s_connecting = pyqtSignal()
    _s_connected = pyqtSignal()
    _s_disconnected = pyqtSignal()
    _s_connection_error = pyqtSignal()
    _s_on_new_data = pyqtSignal()
    _s_update_sensor_settings_ui = pyqtSignal()

    PACKET_SIZE = 640
    UPDATE_FREQUENCY = 30

    DEFAULT_IP = '192.168.6.83'
    DEFAULT_PORT = '5000'
    DEFAULT_NFR_ADDRESS = '0/90/2M/E7E7E7E7E7'

    _data_links = {dlink.name: dlink for dlink in DataLinkType}

    def __init__(self, tabWidget, helper, *args):
        super(SlamdeckTab, self).__init__(*args)
        self.setupUi(self)
        self.tabWidget = tabWidget

        self.vbat: float = 0.0

        # Create sensor and crazyflie models.
        self._model_sensors = [ModelVL53L5CX(id=id.value) for id in SlamdeckSensorId]
        self._model_cf = ModelCrazyflie()
        self._data_link = DataLinkType.NRF_CRTP

        self._slamdeck = Slamdeck(self._data_link, self._model_sensors, self._model_cf, self.set_vbat)

        # Get references to children Qt elements
        self.comboSensor: QComboBox = self.findChild(QComboBox, 'comboSensor')
        self.comboPowerMode: QComboBox = self.findChild(QComboBox, 'comboPowerMode')
        self.comboResolution: QComboBox = self.findChild(QComboBox, 'comboResolution')
        self.editRangingFrequency: QLineEdit = self.findChild(QLineEdit, 'editRangingFrequency')
        self.editIntegrationTime: QLineEdit = self.findChild(QLineEdit, 'editIntegrationTime')
        self.editSharpener: QLineEdit = self.findChild(QLineEdit, 'editSharpener')
        self.comboTargerOrder: QComboBox = self.findChild(QComboBox, 'comboTargerOrder')
        self.comboRangingMode: QComboBox = self.findChild(QComboBox, 'comboRangingMode')
        self.mainTabs: QTabWidget = self.findChild(QTabWidget, 'mainTabs')

        self.graphics2dPointCloud: QGraphicsView = self.findChild(QGraphicsView, 'graphics2dPointCloud')
        self.graphics3dVisualizer: QFrame = self.findChild(QFrame, 'graphics3dVisualizer')
        self.btnSensor = self.findChild(QPushButton, 'btnSetSensor')
        self.comboDataLink: QComboBox = self.findChild(QComboBox, 'comboDataLink')
        self.editPort = self.findChild(QLineEdit, 'editPort')
        self.editIp = self.findChild(QLineEdit, 'editIp')
        self.editRadioAddress = self.findChild(QLineEdit, 'editRadioAddress')

        # Create tables to convert strings to class objects
        self._string_to_sensor = {sensor.name: sensor.value for sensor in SlamdeckSensorId}
        self._string_to_power_mode = {mode.name: mode.value for mode in VL53L5CX_PowerMode}
        self._string_to_resolution = {res.name: res.value for res in VL53L5CX_Resolution}
        self._string_to_target_order = {target.name: target.value for target in VL53L5CX_TargetOrder}
        self._string_to_ranging_mode = {mode.name: mode.value for mode in VL53L5CX_RangingMode}

        # --- Data visualizers --- #

        # 2D Matrix
        self.sensor_single = Visualizer2dMatrix(self.singleSensor, self._model_sensors[SlamdeckSensorId.BACK])
        self.sensor_front = Visualizer2dMatrix(self.sensorFront, self._model_sensors[SlamdeckSensorId.FRONT], small=True)
        self.sensor_main = Visualizer2dMatrix(self.sensorMain, self._model_sensors[SlamdeckSensorId.MAIN], small=True)
        self.sensor_right = Visualizer2dMatrix(self.sensorRight, self._model_sensors[SlamdeckSensorId.RIGHT], small=True)
        self.sensor_back = Visualizer2dMatrix(self.sensorBack, self._model_sensors[SlamdeckSensorId.BACK], small=True)
        self.sensor_left = Visualizer2dMatrix(self.sensorLeft, self._model_sensors[SlamdeckSensorId.LEFT], small=True)

        # 2D Point cloud
        self.visualizer_2d_point_cloud = Visualizer2dPointCloud(self._model_sensors, self._model_cf, self.checkBoxLayout, None)
        #lay = QVBoxLayout(self.graphics3dVisualizer)
        self.layout2dPointCloud.addWidget(self.visualizer_2d_point_cloud.native)

        # 3d Point cloud
        self.visualizer_3d = Visualizer3d(self._model_sensors, self._model_cf)
        lay = QVBoxLayout(self.graphics3dVisualizer)
        lay.addWidget(self.visualizer_3d.native)

        # Attach flight controller. TODO: Fix private access...
        self._fc = FlightController(self._slamdeck._cf, {
            'x': self.fcX,
            'y': self.fcY,
            'z': self.fcZ,
            'yaw': self.fcYaw,
            'mode': self.fcMode
        })
        self.fcStart.clicked.connect(self._fc.start)
        self.fcStop.clicked.connect(self._fc.stop)

        # -- Attach callbacks -- #

        # Slamdeck connect/disconnect
        self._slamdeck.cb_connecting.add_callback(self._s_connecting.emit)
        self._slamdeck.cb_connected.add_callback(self._s_connected.emit)
        self._slamdeck.cb_error.add_callback(self._s_connection_error.emit)
        self._slamdeck.cb_disconnected.add_callback(self._s_disconnected.emit)

        # Buttons
        self.btnSensor.clicked.connect(self._set_settings)
        self.btnConnect.clicked.connect(self._connect)
        self.btnDisconnect.clicked.connect(self._disconnect)
        self.btnStartStreaming.clicked.connect(self._start_streaming)
        self.btnStopStreaming.clicked.connect(self._stop_streaming)

        self._s_on_new_data.connect(self._cb_on_new_data)
        self._s_connecting.connect(self._cb_connecting)
        self._s_connected.connect(self._cb_connected)
        self._s_disconnected.connect(self._cb_disconnected)
        self._s_connection_error.connect(self._cb_connection_error)
        self._s_update_sensor_settings_ui.connect(self._cb_update_sensor_settings_ui)

        self.comboSensor.currentTextChanged.connect(self._sensor_changed)
        self.comboDataLink.currentTextChanged.connect(self._data_links_changed)

        # Update UI before we start
        self._update_button_states(connected=False)
        self._populate_comboboxes()
        self._set_default_values()
        self._data_links_changed()
        self._s_update_sensor_settings_ui.emit()


        self.ui_timer = utils.start_timer(self._update_ui, self.UPDATE_FREQUENCY)

        from PyQt5.QtCore import Qt
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def set_vbat(self, vbat: float):
        self.vbat = vbat

    def keyPressEvent(self, event):
        self._fc.on_key_press(event)

    def keyReleaseEvent(self, event):
        self._fc.on_key_release(event)

    def _sensor_changed(self) -> None:
        self.sensor_single.set_sensor(self._current_sensor())

    def _data_links_changed(self) -> None:
        self._data_link = self._data_links[self.comboDataLink.currentText()]
        self.editIp.setEnabled(False)
        self.editPort.setEnabled(False)
        self.editRadioAddress.setEnabled(False)

        if self._data_link == DataLinkType.SOCKET_TCP:
            self.editIp.setEnabled(True)
            self.editPort.setEnabled(True)
        elif self._data_link == DataLinkType.NRF_CRTP:
            self.editRadioAddress.setEnabled(True)

    def _current_sensor(self) -> SlamdeckSensorId:
        sensor_id = SlamdeckSensorId(self._string_to_sensor[self.comboSensor.currentText()])
        return self._model_sensors[sensor_id]

    def _get_ip(self) -> str:
        return self.editIp.text()

    def _get_port(self) -> int:
        return int(self.editPort.text())

    def _get_nrf_address(self) -> str:
        return self.editRadioAddress.text()

    def _set_default_values(self) -> None:
        self.editIp.setText(self.DEFAULT_IP)
        self.editPort.setText(self.DEFAULT_PORT)
        self.editRadioAddress.setText(self.DEFAULT_NFR_ADDRESS)

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
        self._slamdeck.set_settings(settings)

    def _populate_comboboxes(self) -> None:
        # Settings
        self.comboSensor.addItems(self._string_to_sensor.keys())
        self.comboPowerMode.addItems(self._string_to_power_mode.keys())
        self.comboResolution.addItems(self._string_to_resolution.keys())
        self.comboTargerOrder.addItems(self._string_to_target_order.keys())
        self.comboRangingMode.addItems(self._string_to_ranging_mode.keys())

        # comboDataLink modes
        self.comboDataLink.addItems(self._data_links.keys())

    def _start_streaming(self) -> None:
        self._slamdeck.start_streaming()

    def _stop_streaming(self) -> None:
        self._slamdeck.stop_streaming()

    def _test(self) -> None:
        self._slamdeck.get_data()

    def _connect(self) -> None:
        if self._slamdeck.is_connected():
            logger.warning('Already connected')
            return

        self.i = 0
        self.t0 = 0

        self._slamdeck.set_data_link(self._data_link)

        if self._data_link == DataLinkType.SOCKET_TCP:
            self._slamdeck.connect(self._get_ip(), self._get_port())
        elif self._data_link == DataLinkType.NRF_CRTP:
            self._slamdeck.connect(self._get_nrf_address())

    def _disconnect(self) -> None:
        self._slamdeck.disconnect()

    def _update_ui(self) -> None:
        self.labelFrameRate.setText(f'{self._slamdeck.get_frame_rate()} frames/sec')
        self.labelDataRate.setText(f'{self._slamdeck.get_data_rate()} bytes/sec')
        self.labelBattery.setText(f'{self.vbat:.3} V')

    def _update_button_states(self, connected: bool) -> None:
        self.btnDisconnect.setEnabled(connected)
        self.btnConnect.setEnabled(not connected)
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
