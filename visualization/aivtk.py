import cv2
import numpy as np
import vtk

class aiWorld (object):
    def __init__ (self, size):
        self.views = []
        self.vtk_renderers = []

        self.win = vtk.vtkRenderWindow()
        self.win.StereoCapableWindowOff()
        self.win.SetSize(*size)

        self.ren_interactor = vtk.vtkRenderWindowInteractor()
        self.ren_interactor.SetRenderWindow(self.win)
        self.ren_interactor.Initialize()

        self._update_cb = None

        bg_ren = aiRenderer()
        bg_ren.interactive = False
        self.add_renderer(bg_ren)

    @property
    def update_cb (self):
        return self._update_cb
    @update_cb.setter
    def update_cb (self, cb):
        self._update_cb = cb
        self.ren_interactor.AddObserver('TimerEvent', cb)
        self.ren_interactor.CreateRepeatingTimer(100)

    def add_renderer (self, ai_ren):
        self.vtk_renderers.append(ai_ren)
        self.win.AddRenderer(ai_ren.ren)
        ai_ren.world = self

    def start (self):
        if self.update_cb == None:
            raise Exception('Before starting the world, add a callback ' + \
                            'that updates the view')
        else:
            self.ren_interactor.Start()


class aiRenderer (object):
    def __init__ (self):
        self.ren = vtk.vtkRenderer()
        self.vtk_objects = []

        self._position = (0, 0, 1, 1)
        self._color = (0, 0, 0)

        self.ren.SetViewport(*self._position)
        self.ren.SetBackground(*self._color)

        self._interactive = True
        self._world = None

    @property
    def interactive(self):
        return self._interactive
    @interactive.setter
    def interactive(self, value):
        self._interactive = value
        self.ren.SetInteractive(self.interactive)

    @property
    def color(self):
        return self._color
    @color.setter
    def color(self, value):
        self._color = value
        self.ren.SetBackground(*self._color)

    @property
    def position(self):
        return self._position
    @position.setter
    def position(self, value):
        self._position = value
        self.ren.SetViewport(*self.position)
