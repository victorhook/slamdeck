import tkinter as tk
from vl53lcx_ui import SensorCanvas
from vl53lcx_backend import Reading, SensorParams, Resolution


class Gui(tk.Tk):

    def __init__(self):
        super().__init__()
        self._init_ui()

        self.sensor1 = SensorCanvas(self, SensorParams(Resolution.RES_4x4, 50))
        self.sensor1.pack()

        self.btn = tk.Button(self, text='Test', command=self._test)
        self.btn.pack()

        self._test()

        self.mainloop()

    def _test(self) -> None:
        self.sensor1.update_grids(Reading([i**3 for i in range(16)]))

    def _init_ui(self) -> None:
        self.title('Slamdeck gui')


if __name__ == '__main__':
    Gui()