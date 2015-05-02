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
        self.renderers = Bunch()
        self.vtk_to_ren_map = {}

        self.win = vtk.vtkRenderWindow()
        self.win.StereoCapableWindowOff()
        self.win.SetSize(*size)

        self.ren_interactor = vtk.vtkRenderWindowInteractor()
        self.ren_interactor.SetRenderWindow(self.win)
        self.ren_interactor.Initialize()

        self.picker = vtk.vtkPointPicker()
        self.picker_tolerance = 0.01
        self.ren_interactor.SetPicker(self.picker)

        self.interactor_style = aiInteractorStyle(self)
        self.ren_interactor.SetInteractorStyle(self.interactor_style)

        self._update_cb = None

        bg_ren = aiRenderer()
        bg_ren.interactive = False
        self.addRenderer(background = bg_ren)

    @property
    def picker_tolerance (self):
        return self._picker_tol
    @picker_tolerance.setter
    def picker_tolerance (self, value):
        self._picker_tol = value
        self.picker.SetTolerance(self._picker_tol)

    @property
    def update_cb (self):
        """ A function that is called on a timer (for now at 60Hz)

        ex:
        def update (renderers):
            print renderers.cloud_ren.car

        renderers is a list of all renderers in the world
        This makes accessing objects in the update method easy

        """
        return self._update_cb
    @update_cb.setter
    def update_cb (self, cb):
        """ The cb field can either be a callback to run at 60Hz or a tuple
        containing the callback and the frequency

        ex:
        world.update_cb = update
        or
        world.update_cb = (update, 10)
        """
        if type(cb) in [list,tuple]:
            cb, freq = cb
        else:
            cb, freq = cb, 60

        if self._update_cb is not None:
            self.ren_interactor.RemoveObserver(self._update_cb)

        self._update_cb = cb
        self.ren_interactor.AddObserver('TimerEvent', self._ai_update_cb)
        self.ren_interactor.CreateRepeatingTimer(1000/freq)

    def _ai_update_cb(self, ren_interactor, event):
        all_renderers = self.renderers
        self.update_cb(all_renderers)

        ren_interactor.GetRenderWindow().Render()

    def addRenderer (self, **kwargs):
        """ Adds a renderer view to the world

        ex:
        ren = aiRenderer()
        world.addRenderer(pretty_renderer=ren)
        print world.renderers.pretty_renderer.objects
        """
        for ren_name, ai_ren in kwargs.iteritems():
            self.renderers[ren_name] = ai_ren
            self.vtk_to_ren_map[ai_ren.ren] = ai_ren
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

class aiInteractorStyle (vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, ai_world):
        self.world = ai_world

        # Maps events to their default function and a renderer specific custom
        # function. See the mouseHandler function description for an example
        self.mouse_events_map = {
            'LeftButtonPressEvent':
            [self.OnLeftButtonDown, 'leftPress'],
            'RightButtonPressEvent':
            [self.OnRightButtonDown, 'rightPress'],
            'LeftButtonReleaseEvent':
            [self.OnLeftButtonUp, 'leftRelease'],
            'RightButtonReleaseEvent':
            [self.OnRightButtonUp, 'rightRelease'],
            'MouseWheelForwardEvent':
            [self.OnMouseWheelForward, 'wheelForward'],
            'MouseWheelForwardEvent':
            [self.OnMouseWheelBackward, 'wheelBackward'],
            'MouseMoveEvent':
            [self.OnMouseMove, 'mouseMove']
        }
        # You should generally use CharEvent. One exception is in an
        # application where the user to hold down a key before being allowed to
        # perform an action
        #
        # The event structure looks like:
        # KeyPressEvent, CharEvent, ..., CharEvent, KeyReleaseEvent
        self.key_events_map = {
            'CharEvent':
            [self.OnChar, 'charEntered'],
            'KeyPressEvent':
            [self.OnKeyPress, 'keyPressed'],
            'KeyReleaseEvent':
            [self.OnKeyRelease, 'keyReleased']
        }

        for event in self.mouse_events_map.keys():
            self.AddObserver(event, self._mouseHandler)
        for event in self.key_events_map.keys():
            self.AddObserver(event, self._keyHandler)

    def _mouseHandler (self, obj, event):
        """Handle mouse interactions. Use the mouse position to choose which renderer
        to pass the event to. If the renderer has a custom mouse callback
        event, call that method, otherwise do the default action. See
        mouse_events_map for a list of all custom event names.

        Ex: Using this line will print the actor when the user left-clicks in
        ren

        # Gives the x,y position of the click, an ai_object and index of the
        # click in that ai_object's actor if an object was picked (see
        # world.picker_tolerance) otherwise None and -1, and a function to run to
        # use the default behaivor
        def custom_handler (x, y, ai_obj, index, default_handler):
            print ai_obj
        ren.mouse_handler.leftPress = custom_handler

        """
        ren_interactor = self.world.ren_interactor
        x, y = ren_interactor.GetEventPosition()
        self.FindPokedRenderer(x, y)
        vtk_renderer = self.GetCurrentRenderer()
        renderer = self.world.vtk_to_ren_map[vtk_renderer]

        default_handler, custom_handler_name = self.mouse_events_map[event]

        if custom_handler_name in renderer.mouse_handler:
            self.world.picker.Pick(x, y, 0, vtk_renderer)
            index = self.world.picker.GetPointId()
            actor = self.world.picker.GetActor()
            if actor is not None:
                ai_object = renderer.vtk_to_object_map[actor]
            else:
                ai_object = None

            custom_handler = renderer.mouse_handler[custom_handler_name]
            custom_handler(x, y, ai_object, index, default_handler)
        else:
            default_handler()

    def _keyHandler (self, obj, event):
        """Handle key interactions. Sends a key event to all renderers. If none of the
        world's renderers have a key binding callback, use the default key
        handler. See key_events_map for a list of all custom event names.

        # Gives the key, and control/alt buttons pressed when the key is hit. It
        # also gives the renderer whose key_press handler managed the event.
        #
        # Key symbol names are declared in https://github.com/Kitware/VTK/
        # in GUISupport/Qt/QVTKInteractorAdapter.cxx
        #
        # BUG: For some reason, the key is reported as None when pressed at the
        # same time as another key.
        def custom_handler (key_sym, ctrl, alt, renderer, default_handler):
            print key_sym
        ren.key_handler.charEntered = custom_handler

        """
        ren_interactor = self.world.ren_interactor
        alt = ren_interactor.GetAltKey()
        ctrl = ren_interactor.GetControlKey()
        key = ren_interactor.GetKeySym()
        default_handler, custom_handler_name = self.key_events_map[event]

        custom = False
        for renderer in self.world.renderers.values():
            if custom_handler_name in renderer.key_handler:
                custom = True
                custom_handler = renderer.key_handler[custom_handler_name]
                custom_handler(key, ctrl, alt, renderer, default_handler)

        # If none of the renderers have a char event callback, run the default handler
        if custom == False:
            default_handler()


class aiRenderer (object):
    def __init__ (self):
        """Creates a renderer view. A view contains aiObjects. By default, aiRenderers
        fill the entire screen and have a black background

        """
        self.ren = vtk.vtkRenderer()
        self.objects = Bunch()
        self.vtk_to_object_map = {}

        # Handles events
        # leftPress, rightPress, leftRelease, rightRelease, wheelForward,
        # wheelBackward, mouseMove
        self.mouse_handler = Bunch()
        # Handles events
        # charEntered, keyPressed, keyReleased
        self.key_handler = Bunch()

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
                self.vtk_to_object_map[ai_obj.actor] = ai_obj
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
        del self.vtk_to_object_map[obj.actor]

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
        self._data = data
        self.ren = None
        self.category = None
        self._color = None
        self._wireframe = False
        self._transform = np.eye(4)

    def modified (self):
        """ Call the function whenever the underlying data is changed """
        self.actor.GetMapper().GetInput().Modified()

    @property
    def data (self):
        return self._data
    @data.setter
    def data (self, pos):
        self._data = pos
        self.modified()

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
        self.CreateFromArray(self.data)
        self._data = self.points_npy

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
        self.tree = cKDTree(data[:, :3])

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
                           [xmax, ymax, zmax]]).astype(np.double)
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

    @property
    def data (self):
        return self._data
    @data.setter
    def data (self, pos):
        """ Since box.data is not tied directly to the image, we must modify
        the source """
        self._data = pos
        bounds = np.vstack((self._data[0], self._data[-1])).T.flatten()
        self.source.SetBounds(bounds)
        self.modified()

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
