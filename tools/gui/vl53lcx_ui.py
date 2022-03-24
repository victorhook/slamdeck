import tkinter as tk
import typing as t
from  matplotlib.colors import LinearSegmentedColormap, rgb2hex
from vl53lcx_backend import SensorParams, Reading


class SensorCanvas(tk.Canvas):

    def __init__(self, master, params: SensorParams) -> None:
        super().__init__(master)
        self.params = params
        self.grids = self._init_ui()
        self.cmap = LinearSegmentedColormap.from_list('rg',["r", "w", "g"], N=self.params.MAX_RANGE)


    def _init_ui(self) -> t.List[t.List['Grid']]:
        grids = []
        for row in range(self.params.resolution):
            grids.append([])
            for col in range(self.params.resolution):
                grid = self.Grid(self, row, col, self.params.grid_size)
                grids[row].append(grid)
        return grids

    def update_grids(self, reading: Reading) -> None:
        grid_nbr = 0
        for row in range(self.params.resolution):
            for col in range(self.params.resolution):
                grid = self.grids[row][col]
                value = reading.values[grid_nbr]
                self.set_grid_value(grid, value)
                grid_nbr += 1

    def set_grid_value(self, grid: 'Grid', value: int) -> None:
        precentage = (value / self.params.MAX_RANGE) * 100
        color = self.cmap(value)
        self.itemconfigure(grid.text, text=str(value))
        self.itemconfigure(grid.rect, fill=rgb2hex(color))


    class Grid:

        def __init__(self, canvas: tk.Canvas, row: int, col: int, grid_size: int) -> None:
            self.canvas = canvas
            self.grid_size = grid_size
            self.rect, self.text = self._init_ui(row, col)

        def _init_ui(self, row: int, col: int) -> t.Tuple[int, int]:
            x, y = self.grid_size*row, self.grid_size*col
            rect_id = self.canvas.create_rectangle(x, y, x+self.grid_size, y+self.grid_size, outline='black', fill='white')
            text_id = self.canvas.create_text(x+self.grid_size/2, y+self.grid_size/2, text='HEY')
            return rect_id, text_id
