import cv2
import numpy as np
from scipy.spatial import cKDTree
import vtk
from vtk_visualizer.pointobject import VTKObject # pip install vtk_visualizer

def array2vtkTransform(self,arr):
    T = vtk.vtkTransform()
    matrix = vtk.vtkMatrix4x4()
    for i in range(0,4):
        for j in range(0,4):
            matrix.SetElement(i, j, arr[i,j])
    T.SetMatrix(matrix)
    return T

class Bunch(dict):
    """ A Bunch is a class that allows javascript-like dictionary creation

    ex:
    > bunch = Bunch(a='hello')
    > print bunch.a
    >>> 'hello'
    > print bunch['a']
    >>> 'hello'

    """
    def __init__(self,**kw):
        dict.__init__(self,kw)
        self.__dict__ = self

    def __str__(self):
        state = ["%s=%r" % (attribute, value) \
                 for (attribute, value) in \
                 self.__dict__.items()]
        return '\n'.join(state)

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
        self.objects = Bunch()

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

    def addObjects (self, **kwargs):
        """ Adds an actor to the renderer

        ex:
        clouds = [aiCloud(np.random.rand(100, 3))] * 3
        ren.addObject(lanes=clouds)
        ren.objects.lanes[0].color = np.random.rand(100, 3) * 255
        """
        # Look through all of the keyword arguments
        for category, ai_objs in kwargs.iteritems():
            # If the value is not a list/tuple, make it one so we can iterate
            # later. This also makes deleting easier
            if type(ai_objs) not in [list,tuple]:
                ai_objs = [ai_objs]

            if category in self.objects.keys():
                # If the category already exists, add the objects to the
                # existing category
                self.objects[category].extend(ai_objs)
            else:
                # Update the Bunch to contain the new name
                # We can access this item by ren.name
                self.objects[category] = ai_objs
            # Add the objects to the renderer and let the object know it has a
            # renderer
            for ai_obj in ai_objs:
                self.ren.AddActor(ai_obj.actor)
                ai_obj.ren = self
                ai_obj.category = category

    def _cleanupObject (self, index, obj):
        """Does all the necessary cleanup for objects in the renderer. If there are no
        objects in a particular category left, delete the category

        """
        category = obj.category
        similar_objects = self.objects[category]
        del similar_objects[index]
        self.ren.RemoveActor(obj.actor)
        # If the list is empty, remove the entire category
        if len(similar_objects) == 0:
            del self.objects[category]

    def removeObjects (self, objects):
        """ Removes a list of objects from the renderer. Objects must be from
        the same category and the category must be in the renderer

        ex:
        boxes = [aiBox((0, 1, 0, 1, 0, 1))] * 10
        ren.addObjects(boxes = boxes)
        ren.removeObjects(ren.objects.boxes[2:4])
        """
        if objects == None:
            return
        if type(objects) not in [list,tuple]:
            # Make the object a list so we can iterate over it
            objects = [objects]
        if len(set([o.category for o in objects])) > 1:
            raise Exception('Objects must belong to the same category')

        category = objects[0].category
        if category not in self.objects.keys():
            raise Exception('Category {category} is not in this renderer'.\
                            format(c=category))

        # Iterate backwards so we can delete
        similar_objects = self.objects[objects[0].category]
        for i in xrange(len(similar_objects) - 1, -1, -1):
            obj = similar_objects[i]
            if obj in objects:
                self._cleanupObject(i, obj)

class aiObject (VTKObject, object):
    def __init__ (self, data):
        self.data = data
        self.ren = None
        self.category = None
        self._color = None
        self._wireframe = False
        self._transform = np.eye(4)

    @property
    def source (self):
        """ Gets the source of the object """
        return self.actor.GetMapper().GetInputConnection(0, 0).GetProducer()

    @property
    def properties (self):
        """ Gets all object properties """
        return self.actor.GetProperty()

    @property
    def point_size (self):
        return self.properties.GetPointSize()
    @point_size.setter
    def point_size (self, value):
        """ Set the actor's point size """
        self.properties.SetPointSize(value)

    @property
    def opacity (self):
        return self.properties.GetOpacity()
    @opacity.setter
    def opacity (self, value):
        """ Change the object's opacity. Takes values 0-1 """
        self.properties.SetOpacity(value)

    @property
    def pickable (self):
        self.actor.GetPickable()
    @pickable.setter
    def pickable (self, value):
        """ Allow the object to be selected by the mouse """
        self.actor.SetPickable(int(value))

    @property
    def visable (self):
        return self.actor.GetVisability()
    @visable.setter
    def visable (self, value):
        """ Change the object's visability. value is a boolean """
        self.actor.SetVisability(int(value))

    @property
    def transform (self):
        return self._transform
    @transform.setter
    def transform (self, np_xform):
        self._transform = np_xform
        self.actor.SetUserTransform(array2vtkTransform(np_xform))

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
        return self.data_color
    @color.setter
    def color (self, color):
        self.data_color = color

class aiKDCloud (aiCloud):
    def __init__ (self, data):
        super(aiKDCloud, self).__init__(data)
        self.tree  = cKDTree(data[:, :3])

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
        return self.actor_color
    @color.setter
    def color (self, color):
        self.actor_color = color

class aiPly(aiObject):
    def __init__ (self, ply_file_name):
        """ Creates an object from a ply file. """
        super(aiPly, self).__init__(np.array((0,0,0)))

        reader = vtk.vtkPLYReader()
        reader.SetFileName(ply_file_name)
        reader.Update()
        ply_mapper = vtk.vtkPolyDataMapper()
        ply_mapper.SetInputConnection(reader.GetOutputPort())
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(ply_mapper)
