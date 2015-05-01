import cv2
import numpy as np
import vtk

class aiWorld (object):
    def __init__ (self, size):
        """ Creates a window with a black background """
        self.views = []
        self.ai_renderers = []

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
        """ A function that is called on a timer (for now at 60Hz)

        ex: def update (ren_interactor, event)
        ren_interactor is of type vtkRenderWindowInteractor

        TODO: make modifying ren_interactor easy
        """
        return self._update_cb
    @update_cb.setter
    def update_cb (self, cb):
        if self._update_cb is not None:
            self.ren_interactor.RemoveObserver(self._update_cb)

        self._update_cb = cb
        self.ren_interactor.AddObserver('TimerEvent', self._update_cb)
        self.ren_interactor.CreateRepeatingTimer(16)

    def add_renderer (self, ai_ren):
        """ Adds a renderer view to the world """
        self.ai_renderers.append(ai_ren)
        self.win.AddRenderer(ai_ren.ren)
        ai_ren.world = self

    def start (self):
        """ Starts running the update callback. If there is no update method, throw an
        error

        """
        if self.update_cb == None:
            raise Exception('Before starting the world, add a callback ' + \
                            'that updates the view')
        else:
            self.ren_interactor.Start()


class aiRenderer (object):
    def __init__ (self):
        """Creates a renderer view. A view contains aiObjects. By default, aiRenderers
        fill the entire screen and have a black background

        """
        self.ren = vtk.vtkRenderer()
        self.ai_objects = []

        self._position = (0, 0, 1, 1)
        self._color = (0, 0, 0)

        self.ren.SetViewport(*self._position)
        self.ren.SetBackground(*self._color)

        self._interactive = True
        self._world = None

    @property
    def interactive (self):
        """Renders can be interactive (register mouse/key clicks) or non-interactive
        (ignore mouse/key clicks)

        """
        return self._interactive
    @interactive.setter
    def interactive (self, value):
        self._interactive = value
        self.ren.InteractiveOff()
        self.ren.SetInteractive(self.interactive)

    @property
    def color (self):
        """Sets the background color

        ex: ren.color = (R, G, B)

        """
        return self._color
    @color.setter
    def color (self, value):
        self._color = value
        self.ren.SetBackground(*self._color)

    @property
    def position (self):
        """Sets the position of the renderer

        ex: This will fill the screen from the bottom-right corner to the
        center of the screen in Y and 3/4 of the screen in X"
        -------------
        |           |
        |xxxxxxxxx  |
        |xxxxxxxxx  |
        -------------

        ren.position = (0, 0, 0.75, 0.5)

        """
        return self._position
    @position.setter
    def position (self, value):
        self._position = value
        self.ren.SetViewport(*self.position)
