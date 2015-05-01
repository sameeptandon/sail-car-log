import cv2
import numpy as np
import vtk
from vtk_visualizer.pointobject import VTKObject # pip install vtk_visualizer

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
        self.addRenderer(bg_ren)

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

    def addRenderer (self, ai_ren):
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
        TODO: Why isn't this working

        """
        return self._interactive
    @interactive.setter
    def interactive (self, value):
        self._interactive = value
        self.ren.SetInteractive(self._interactive)

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

    def addObject (self, ai_object):
        """ Adds an actor to the renderer """
        self.ai_objects.append(ai_object)
        self.ren.AddActor(ai_object.actor)
        ai_object.ren = self


class aiObject (VTKObject, object):
# SetPickable, SetPointSize, SetOpacity, kdtree?
# SetColor, RotateZ (Rotate?), SetOpacity, SetVisability
    def __init__ (self, data):
        self.data = data
        self._color = None
        self.ren = None
        self._wireframe = False

    @property
    def source (self):
        """ Gets the source of the object """
        return self.actor.GetMapper().GetInputConnection(0, 0).GetProducer()

    @property
    def properties (self):
        """ Gets all object properties """
        return self.actor.GetProperty()

    @property
    def wireframe (self):
        """ The object is displayed as a wireframe """
        return self._wireframe
    @wireframe.setter
    def wireframe (self, value):
        self._wireframe = value
        if value == True:
            self.properties.SetRepresentationToWireframe()
        else:
            self.properties.SetRepresentationToSurface()

    @property
    def data_color (self):
        """ Sets the color of the data source """
        return self._color
    @data_color.setter
    def data_color (self, color):
        """ Colors is a numpy array the same size as data in the form:
        [[r, g, b]...[r,g,b]] taking values 0-255.
        Parent classes should choose how setting the color changes the object
        """
        self._color = color
        self.AddColors(self._color)

    @property
    def actor_color (self):
        """ Sets the color of the entire object """
        return self._color
    @actor_color.setter
    def actor_color (self, color):
        """ Colors is a tuple (R, G, B) taking values 0-255.
        Parent classes should choose how setting the color changes the object
        """
        (R, G, B) = color
        self._color = color
        self.properties.SetColor(R/255., G/255., B/255.)

    def projectData (self):
        """ Project the data into 2d """
        pass

    def moveData (self):
        """ Move part of the data around """
        pass

    def transformData (self):
        """ Move the data to a different frame of reference """
        pass

class aiCloud (aiObject):
    def __init__ (self, data):
        super(aiCloud, self).__init__(data)
        self.data = self.CreateFromArray(self.data)

    # All points can have unique colors
    @property
    def color (self):
        return self._color
    @color.setter
    def color (self, color):
        self.data_color = color

class aiBox(aiObject):
    def __init__ (self, bounds):
        """Create a box witih the given bounds [xmin,xmax,ymin,ymax,zmin,zmax]"""
        self.bounds = bounds
        (xmin, xmax, ymin, ymax, zmin, zmax) = self.bounds
        coords = np.array([[xmin, ymin, zmin],
                           [xmin, ymin, zmax],
                           [xmin, ymax, zmin],
                           [xmin, ymax, zmax],
                           [xmax, ymin, zmin],
                           [xmax, ymin, zmax],
                           [xmax, ymax, zmin],
                           [xmax, ymax, zmax]])
        super(aiBox, self).__init__(coords)

        self.CreateBox(bounds)

        self.properties.LightingOff()
        self.source.QuadsOn()

        self.wireframe = True

    # A box's color is defined by the actor, not the points
    @property
    def color (self):
        return self._color
    @color.setter
    def color (self, color):
        self.actor_color = color
