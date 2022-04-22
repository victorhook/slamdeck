from dataclasses import dataclass
import numpy as np
import typing as t
from collections import namedtuple

from PyQt5.QtCore import pyqtSignal, QPointF, QSizeF, QRectF, QObject, QGraphicsScene, QGraphicsView, QGraphicsTextItem, QGraphicsRectItem, QGraphicsEllipseItem
from PyQt5.QtGui import QPen, QColor, qRgb, QTextBlock, QBrush

XYPos = namedtuple('XYPos', ['x', 'y'])

@dataclass
class RadialCoordinate:
    length: float
    angle_degrees: float


@dataclass
class Drone:
    x: float = 0
    y: float = 0
    z: float = 0
    roll_degrees: float = 0
    pitch_degrees: float = 0
    yaw_degrees: float = 0


@dataclass
class DrawableObject2D:
    x: float
    y: float
    color: QColor
    size: float = 10.0
    point: QGraphicsEllipseItem = None


class Visualizer(QObject):

    SIZE_MULTIPLIER = 20
    SENSOR_FOV      = 45
    COLOR_DRONE     = QColor(255, 0, 0, 1)
    COLOR_WALL      = QColor(0, 0, 0, 1)
    _s_update_ui    = pyqtSignal(list)

    def __init__(self, graphics: QGraphicsView):
        self.drone = Drone()
        self.graphics = graphics
        self.scene = QGraphicsScene()
        self.graphics.setScene(self.scene)
        self._s_update_ui.connect(self._update_ui)

        #self.drawable_drone = DrawableObject2D(0, 0, self.COLOR_DRONE)
        #self.drawable_environment = [DrawableObject2D(0, 0, self.COLOR_WALL) for x, y in xy_pos_2d] += DrawableObject2D(self.drone.x, self.drone.y, self.COLOR_DRONE)

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

    def _get_2d_radial_coordinates(self, distances: t.List[np.uint16], grid_per_row: int, drone: Drone) -> t.List[RadialCoordinate]:
        radial_distances = [RadialCoordinate(0, 0) for i in range(grid_per_row)]
        grid_fov = self.SENSOR_FOV / grid_per_row
        for grid, distance in enumerate(distances):
            radial_distances[grid].length = distance
            radial_distances[grid].angle_degrees = grid*grid_fov + drone.yaw_degrees
        return radial_distances

    def _get_2d_distances(self, distances: t.List[np.uint16], grid_per_row: int) -> t.List[np.uint16]:
        return distances[::grid_per_row]
