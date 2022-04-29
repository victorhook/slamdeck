import enum
from itertools import chain
from queue import Queue
import PyQt5
from dataclasses import dataclass
import numpy as np
import typing as t
from collections import namedtuple

import PyQt5.QtCore
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsScene, QGraphicsScale, QCheckBox, QSizePolicy, QSpacerItem, QVBoxLayout, QGraphicsView, QGraphicsEllipseItem, QRadioButton
from PyQt5.QtGui import QPen, QColor, qRgb, QTextBlock, QBrush
from gui.models import ModelCrazyflie, ModelVL53L5CX, VL53L5CX_Resolution

from vispy import scene

from gui.slamdeck import SlamdeckSensorId

XYPos = namedtuple('XYPos', ['x', 'y'])

@dataclass
class RadialCoordinate:
    length: float
    angle_degrees: float


@dataclass
class DrawableObject2D:
    x: float
    y: float
    color: QColor
    size: float = 10.0
    point: QGraphicsEllipseItem = None


class Visualizer2dPointCloud(scene.SceneCanvas):

    SIZE_MULTIPLIER = 20
    SENSOR_FOV      = 45
    COLOR_DRONE     = QColor(255, 0, 0, 1)
    COLOR_WALL      = QColor(0, 0, 0, 1)
    COLOR_ACTIVE    = QColor(255, 0, 0)
    COLOR_INACTIVE  = QColor(0, 0, 0)
    _s_update_ui    = pyqtSignal(list)

    FPS = 30
    Y_AXIS_INVERTED = True
    POINT_QUEUE_SIZE = 5

    def __init__(self,
            sensors: t.List[ModelVL53L5CX],
            crazyflie: ModelCrazyflie,
            checkBoxLayout: QVBoxLayout,
            checkBoxGraphics: QGraphicsView,
            graphics: QGraphicsView
        ):
        scene.SceneCanvas.__init__(self, keys=None)
        self.unfreeze()

        # Models
        self._model_sensors = []
        for sensor in sensors:
            if sensor.id != SlamdeckSensorId.MAIN.value:
                self._model_sensors.append(sensor)
        self._model_crazyflie = crazyflie

        self._view = self.central_widget.add_view()
        self._view.bgcolor = '#ffffff'
        self._view.camera = scene.PanZoomCamera(
            rect=(0, 0, self._view.width, self._view.height),
            aspect=1.0,
        )

        # UI
        self.graphics = graphics
        self.checkBoxLayout = checkBoxLayout

        self.checkBoxGraphics = checkBoxGraphics
        self.checkScene = QGraphicsScene()
        self.checkScene.setSceneRect(0, 0, self.checkBoxGraphics.width(), self.checkBoxGraphics.height())
        self.checkBoxGraphics.setScene(self.checkScene)
        self.checkBoxGraphics.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.checkBoxGraphics.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self._check_boxes: t.List[QCheckBox] = []
        self._rows: t.List[QGraphicsRectItem] = []
        self.set_resolution(VL53L5CX_Resolution.RESOLUTION_4X4)
        self.set_resolution(VL53L5CX_Resolution.RESOLUTION_8X8)

        #self.scene = QGraphicsScene()
        #self.graphics.setScene(self.scene)
        #self._s_update_ui.connect(self._update_ui)

        #self.drawable_drone = DrawableObject2D(0, 0, self.COLOR_DRONE)
        #self.drawable_environment = [DrawableObject2D(0, 0, self.COLOR_WALL) for x, y in xy_pos_2d] += DrawableObject2D(self.drone.x, self.drone.y, self.COLOR_DRONE)

        self._graph_timer = QTimer()
        self._graph_timer.setInterval(int(1000 / self.FPS))
        self._graph_timer.timeout.connect(self._update_graphics)
        self._graph_timer.start()

        self.cx = self._view.width / 2
        self.cy = self._view.height / 2

        self.CF_SIZE = 60
        self._sensor_angles = {
            SlamdeckSensorId.RIGHT: 0,
            SlamdeckSensorId.FRONT: 90,
            SlamdeckSensorId.LEFT:  180,
            SlamdeckSensorId.BACK:  270,
        }
        self._cf_lines: scene.visuals.Line
        self._points: scene.visuals.Line
        self._points_buffer = Queue(self.POINT_QUEUE_SIZE)
        self._create_graphics()

        self._update_crazyflie()
        self._update_points()


    def _create_graphics(self) -> None:

        self._cf_lines = scene.visuals.Line(
            pos=np.array([
                [self.cx, self.cy],
                [self.cx+self.CF_SIZE/2, self.cy+self.CF_SIZE/2],
                [self.cx-self.CF_SIZE/2, self.cy+self.CF_SIZE/2],
                [self.cx, self.cy]
            ]),
            parent=self._view.scene,
            width=4,
            method='agg'
        )

        self._points = scene.visuals.Markers(
            pos=np.array([[0, 0], [0, 0]]),
            parent=self._view.scene,
            symbol='o',
            size=1,
            face_color=np.array((1, 1, 1))
        )

        # Axis
        axis_length = 80

        sign = -1 if self.Y_AXIS_INVERTED else 1
        if self.Y_AXIS_INVERTED:
            axis_start = [self._view.width - 20, self._view.height - (20 + axis_length)]
        else:
            axis_start = [self._view.width - 20, 20 + axis_length]

        x_tip = [axis_start[0], axis_start[1] - sign*axis_length]
        y_tip = [axis_start[0] - axis_length, axis_start[1]]
        scene.visuals.Arrow(
            pos=np.array([
                axis_start, x_tip,
                axis_start, y_tip
            ]),
            arrows=np.array([
                axis_start, x_tip,
                axis_start, y_tip
            ]).reshape(-1, 4),
            parent=self._view.scene,
            arrow_size=15.,
            arrow_type='stealth',
            width=3
        )
        x_text = x_tip
        x_text[0] -= 20
        y_text = y_tip
        y_text[1] -= 20
        scene.visuals.Text('x', pos=x_text, parent=self._view.scene)
        scene.visuals.Text('y', pos=y_text, parent=self._view.scene)

    def _update_crazyflie(self) -> None:
        yaw = self._model_crazyflie.yaw + 90
        center = np.array([
            self.cx + self._model_crazyflie.x,
            self.cy - self._model_crazyflie.y
        ])

        angle_p1 = np.deg2rad(yaw)
        angle_p2 = np.deg2rad(yaw - 140)
        angle_p3 = np.deg2rad(yaw + 140)

        sign = -1 if self.Y_AXIS_INVERTED else 1

        cf_data = np.array([
            [self.CF_SIZE*np.cos(angle_p1), sign*-self.CF_SIZE*np.sin(angle_p1)],
            [self.CF_SIZE*np.cos(angle_p2), sign*-self.CF_SIZE*np.sin(angle_p2)],
            [self.CF_SIZE*np.cos(angle_p3), sign*-self.CF_SIZE*np.sin(angle_p3)],
            [self.CF_SIZE*np.cos(angle_p1), sign*-self.CF_SIZE*np.sin(angle_p1)]
        ])

        for point in cf_data:
            point += center

        self._cf_lines.set_data(cf_data)

    def _update_points(self) -> None:
        point_data = []

        rows = self._model_sensors[0].get_row_size()

        for sensor in self._model_sensors:
            angle = self._sensor_angles[sensor.id] + self._model_crazyflie.yaw
            data = self._get_sensor_data(sensor, rows)
            points = self._get_sensor_points(data, rows, angle)
            point_data.extend(points)

        if self._points_buffer.full():
            self._points_buffer.get()

        self._points_buffer.put(point_data)

        if not self._points_buffer.full():
            return

        colors = []
        fade = 1 / self.POINT_QUEUE_SIZE
        for i in range(self.POINT_QUEUE_SIZE):
            for point in range(len(self._points_buffer.queue[i])):
                colors.append((0, 0, 0, 1-(i*fade)))

        points = np.array(list(chain(*self._points_buffer.queue)))
        colors = np.array(colors)

        self._points.set_data(
            pos=points,
            face_color=colors
        )

    def _get_sensor_data(self, sensor: ModelVL53L5CX, size: int) -> np.array:
        """
            Filters the rows of the sensor according to the set checkboxes.
            Then merges the rows for each column, and returns a 1-dimensional
            array that represents the distance data in a given orientation.
        """
        def get_rows(enabled_rows: t.List[int]) -> t.List[t.List[int]]:
            rows = []
            row = 0
            index = 0

            while row < size:

                if row not in enabled_rows:
                    row += 1
                    continue

                row_data = []
                for col in range(size):
                    index = (size-1) - row*size + col
                    row_data.append(sensor.data[index])
                rows.append(row_data)

                row += 1

            return rows

        enabled_rows = self._get_enabled_rows()

        data = [0 for i in range(size)]
        rows = get_rows(enabled_rows)
        # Add all data
        for row in rows:
            for col in range(size):
                data[col] += row[col]

        # Average data
        data = np.array(data) / len(enabled_rows)
        return data

    def _get_sensor_points(self, row: np.array, size: int, angle: int) -> t.List[t.Tuple[float, float]]:
        cfx = self.cx + self._model_crazyflie.x
        cfy = self.cy + self._model_crazyflie.y

        fov = 45.0
        angle_size = fov / size
        angle = angle - (fov / 2) + (angle_size / 2)

        sign = -1 if self.Y_AXIS_INVERTED else 1

        points = []

        # -22.5 -> 22.5
        for distance in row:
            x = distance*np.cos(np.deg2rad(angle))
            y = sign*-1*distance*np.sin(np.deg2rad(angle))

            # Scale down to fit scene
            x /= 10
            y /= 10

            # Translate to cf position
            x += cfx
            y += cfy

            points.append((x, y))
            angle += angle_size

        return points

    def _get_enabled_rows(self) -> t.List[int]:
        enabled_rows = []
        for box in self._check_boxes:
            if box.isChecked():
                # All boxes are given a name on creation with their row.
                enabled_rows.append(int(box.accessibleName()))
        return enabled_rows

    def _update_graphics(self) -> None:
        self._update_crazyflie()
        self._update_points()

    def _checkbox_change(self) -> None:
        for row, check in zip(self._rows, self._check_boxes):
            if check.isChecked():
                color = self.COLOR_ACTIVE
            else:
                color = self.COLOR_INACTIVE

            row.setBrush(QBrush(color))

    def _update_check_boxes(self, buttons: int) -> None:
        for check in self._check_boxes:
            self.checkBoxLayout.removeWidget(check)

        for row in self._rows:
            self.checkScene.removeItem(row)

        width = self.checkBoxGraphics.width()
        height = self.checkBoxGraphics.height() / buttons

        self._check_boxes = []
        self._rows = []
        for btn in reversed(range(buttons)):
            check = QCheckBox(f'Row {btn}')
            # Give box row name for later reference
            check.setAccessibleName(f'{buttons-btn}')
            check.setChecked(True)
            check.stateChanged.connect(self._checkbox_change)
            check.setStyleSheet("QCheckBox::checked"
                                "{"
                                "background-color : lightgreen;"
                                "}")
            self.checkBoxLayout.addWidget(check)
            self._check_boxes.append(check)

            # Create row
            #row = QGraphicsRectItem(0, btn*height, width, height)
            #row.setBrush(QBrush(self.COLOR_ACTIVE))
            #self._rows.append(row)
            #self.checkScene.addItem(row)

        self._checkbox_change()

    def set_resolution(self, resolution: VL53L5CX_Resolution) -> None:
        if resolution == VL53L5CX_Resolution.RESOLUTION_4X4:
            self._update_check_boxes(4)
        else:
            self._update_check_boxes(8)

    def update_data(self,
                    x: float,
                    y: float,
                    yaw_degrees: float,
                    distances: t.Dict[str, t.List[np.uint16]],
                    resolution: np.uint8
        ) -> None:
        self.drone.x = x
        self.drone.y = y
        self.drone.yaw_degrees = yaw_degrees

        if resolution == 16:
            grid_per_row = 4
        elif resolution == 64:
            grid_per_row = 8

        distances_2d_front = self._get_2d_distances(distances['distance_front'], grid_per_row)
        distances_2d_right = self._get_2d_distances(distances['distance_right'], grid_per_row)
        distances_2d_back = self._get_2d_distances(distances['distance_back'], grid_per_row)
        distances_2d_left = self._get_2d_distances(distances['distance_left'], grid_per_row)

        radial_coordinates_2d = []
        radial_coordinates_2d.extend(self._get_2d_radial_coordinates(distances_2d_front, grid_per_row))
        radial_coordinates_2d.extend(self._get_2d_radial_coordinates(distances_2d_right, grid_per_row))
        radial_coordinates_2d.extend(self._get_2d_radial_coordinates(distances_2d_back, grid_per_row))
        radial_coordinates_2d.extend(self._get_2d_radial_coordinates(distances_2d_left, grid_per_row))

        xy_pos_2d = self._get_2d_xy_pos(radial_coordinates_2d)


        self._s_update_ui.emit(drawable_objects)

    def _update_ui(self, objects: t.List[DrawableObject2D]) -> None:
        for obj in objects:
            self._draw_object(obj.x, obj.y, obj.color)

    def _draw_object(self, obj: DrawableObject2D) -> None:
        point = QGraphicsEllipseItem(obj.x, obj.y, obj.size, obj.size)
        point.setBrush(QBrush(point.color))
        self.scene.addItem(point)

    def _get_2d_xy_pos(self, radial_coordinates: t.List[RadialCoordinate]) -> t.List[XYPos]:
        return [
            XYPos(
                x=coord.length * np.cos(coord.angle),
                y=coord.length * np.sin(coord.angle)
            )
            for coord in radial_coordinates
        ]

    def _get_2d_radial_coordinates(self, distances: t.List[np.uint16], grid_per_row: int, drone: object) -> t.List[RadialCoordinate]:
        radial_distances = [RadialCoordinate(0, 0) for i in range(grid_per_row)]
        grid_fov = self.SENSOR_FOV / grid_per_row
        for grid, distance in enumerate(distances):
            radial_distances[grid].length = distance
            radial_distances[grid].angle_degrees = grid*grid_fov + drone.yaw_degrees
        return radial_distances

    def _get_2d_distances(self, distances: t.List[np.uint16], grid_per_row: int) -> t.List[np.uint16]:
        return distances[::grid_per_row]
