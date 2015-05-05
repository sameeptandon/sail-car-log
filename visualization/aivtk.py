import cv2
import numpy as np
from scipy.spatial import cKDTree
import vtk
from vtk_visualizer.pointobject import VTKObject # pip install vtk_visualizer
from vtk.util import numpy_support

class aiArray(np.ndarray):
    """Wrap numpy array in an object that will show updates to the renderer
    whenever the data element of an aiObject is changed. In order for this to
    happen, we need to call ai_obj.modified() whenever the underlying data is
    changed. We can not do this with an @property decorator because slicing the
    array creates a new view.

    For the following example, cloud's data is of type np.ndarray:
    cloud = aiCloud(np.random.rand(num_pts, 3))
    # This will call cloud.modified() and will update in the view
    cloud.data += np.array((1,1,1))
    # Slicing the array calls aiObj's data getter, but not it's setter! The
    # data on the screen would not be updated. The internal data would be
    # updated, but the vtk data would not be
    cloud.data[:, 2] += 10 # This would not normally update

    """
    def __new__(cls, input_array, ai_obj):
        obj = np.asarray(input_array).view(cls)
        obj.ai_obj = ai_obj
        return obj

    def __array_finalize__ (self, obj):
        if obj is None: return
        self.ai_obj = getattr(obj, 'ai_obj', None)

    def __array_wrap__(self, out_arr, context=None):
        # This is the key to updating the data on the screen after slicing
        self.ai_obj.modified()
        return np.ndarray.__array_wrap__(self, out_arr, context)

def array2vtkTransform(arr):
    T = vtk.vtkTransform()
    matrix = vtk.vtkMatrix4x4()
    for i in range(0,4):
        for j in range(0,4):
            matrix.SetElement(i, j, arr[i,j])
    T.SetMatrix(matrix)
    return T

def mkVtkCells (faces):
    polys = vtk.vtkCellArray()
    for i in xrange(len(faces)):
        polys.InsertNextCell(mkVtkIdList(faces[i]))
    return polys

def mkVtkIdList (it):
    vil = vtk.vtkIdList()
    for i in it:
        vil.InsertNextId(int(i))
    return vil

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
        if self.update_cb is None:
            raise Exception('Before starting the world, add a callback ' + \
                            'that updates the view')
        else:
            self.ren_interactor.Start()

    def quit (self):
        """ End the program and kill the window """
        self.ren_interactor.TerminateApp()

class aiInteractorStyle (vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, ai_world):
        self.world = ai_world

        # Maps events to their default function and a renderer specific custom
        # function. See the mouseHandler function description for an example
        self.mouse_events_map = {
            'LeftButtonPressEvent':
            [self.OnLeftButtonDown, 'leftPress'],
            'LeftButtonReleaseEvent':
            [self.OnLeftButtonUp, 'leftRelease'],
            'RightButtonPressEvent':
            [self.OnRightButtonDown, 'rightPress'],
            'RightButtonReleaseEvent':
            [self.OnRightButtonUp, 'rightRelease'],
            'MiddleButtonPressEvent':
            [self.OnMiddleButtonDown, 'middlePress'],
            'MiddleButtonReleaseEvent':
            [self.OnMiddleButtonUp, 'middleRelease'],
            'MouseWheelForwardEvent':
            [self.OnMouseWheelForward, 'wheelForward'],
            'MouseWheelBackwardEvent':
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
        def custom_handler (x, y, ai_obj, index, renderer, default_handler):
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
            custom_handler(x, y, ai_object, index, renderer, default_handler)
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
    def __init__ (self, xmin=0, ymin=0, xmax=1, ymax=1):
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

        # Let the renderer hold some metadata to pass between callbacks
        self.meta = Bunch()

        self.position = (xmin, ymin, xmax, ymax)
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
    def transparent (self):
        return bool(self.ren.GetErase())
    @transparent.setter
    def transparent (self, val):
        """ Sets the render window background to transparent """
        self.ren.SetErase(int(not val))

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

    @property
    def cam (self):
        return self.ren.GetActiveCamera()

    @property
    def cam_position (self):
        return self.cam.GetPosition()
    @cam_position.setter
    def cam_position (self, value):
        return self.cam.SetPosition(value)

    @property
    def cam_focal_point (self):
        return self.cam.GetFocalPoint()
    @cam_focal_point.setter
    def cam_focal_point (self, value):
        return self.cam.SetFocalPoint(value)

    @property
    def cam_view_up (self):
        return self.cam.GetViewUp()
    @cam_view_up.setter
    def cam_view_up (self, value):
        return self.cam.SetViewUp(value)

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
        if objects is None:
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

    def displayToRay (self, x, y, z):
        """ The z coordinate here is relative to the clipping plane. -1 is at
        the camera location and 1 is the clipping plane far away """
        world_point = [0, 0, 0, 0]
        vtk.vtkInteractorObserver.\
              ComputeDisplayToWorld(self.ren, x, y, z, world_point)
        return np.array(world_point)[:3]

    def displayToWorld (self, x, y, X, Y, Z):
        """ Computes a position in 3D given the pixel coordinates and a nearby
        3D point. We use the 3D point to compute the z-distance in the camera
        frame (x-positive: right, y-positive: up, z-positive: out). This is
        useful because the z postion changes depending on the camera
        coordinates. """
        screen_point = [0, 0, 0]

        # Use a world point to calculate depth in the camera
        vtk.vtkInteractorObserver.\
            ComputeWorldToDisplay(self.ren, X, Y, Z, screen_point)
        # Use the mouse position and the old depth to get a new 3d point
        return self.displayToRay(x, y, screen_point[2])

class aiObject (VTKObject, object):
    def __init__ (self, data):
        super(aiObject, self).__init__()

        if len(data.shape) == 1:
            data = data[:, np.newaxis]
        data = data.astype(np.float64)
        self._data = aiArray(data, ai_obj=self)
        self._data_hom = None
        self.ren = None
        self.category = None
        self._color = None
        self._wireframe = False
        self._transform = np.eye(4)
        self.group = [self]

    def modified (self):
        """ Call the function whenever the underlying data is changed """
        self.actor.GetMapper().GetInput().Modified()

    @property
    def data (self):
        # Any time we access data, make sure the verts are synced
        self.verts.SetData(numpy_support.numpy_to_vtk(self._data))
        return self._data
    @data.setter
    def data (self, pos):
        self._data = pos
        # Any time we modify data, make sure the verts are synced
        self.verts.SetData(numpy_support.numpy_to_vtk(self._data))
        self.modified()

    @property
    def data_hom (self):
        """ Creates a homogenous view of the data """
        if self._data_hom is None:
            self._data_hom = np.hstack((self._data, np.ones((self._data.shape[0], 1))))
        return self._data_hom
    @data_hom.setter
    def data_hom (self, data_hom):
        """ Updates the data by using the 4th as a scaling factor
        TODO: is there a better way than copying? """
        self._data_hom = data_hom
        self.data = self._data_hom[:, :3].copy()

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
    def visible (self):
        return self.actor.GetVisibility()
    @visible.setter
    def visible (self, value):
        """ Change the object's visability. value is a boolean """
        self.actor.SetVisibility(int(value))

    @property
    def transform (self):
        return self._transform
    @transform.setter
    def transform (self, np_xform):
        assert (np_xform.shape == (4,4))
        self._transform = np_xform
        # Update the homogenous data (also updates the underlying data)
        self.data_hom = np_xform.dot(self.data_hom.T).T

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
        [[r, g, b]...[r, g, b]] taking values 0-255.
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

    def link (self, ai_obj):
        """ Link the objects together. When we pick an object from the
        renderer, we can access its group to know other objects it should
        interact with. This operation is symmetric. Self is always the first
        object in the group.

        ex:
        obj.link(obj2)
        obj in obj2.group
        >>> True
        obj2 in obj.group
        >>> True
        obj in obj.group
        >>> True
        """
        self.group.append(ai_obj)
        ai_obj.group.append(ai_obj)
    def unlink (self, ai_obj):
        """ Unlink objects from each other. This operation is symmetric. Raises
        an exception if trying to unlink from itself """
        if ai_obj == self:
            raise Exception ("self cannot unlink from self")
        if ai_obj in self.group:
            del self.group[self.group.index(ai_obj)]
        if self in ai_obj.group:
            del ai_obj.group[ai_obj.group.index(self)]

    def projectData (self):
        """ Project the data into 2d """
        pass

class aiCloud (aiObject):
    def __init__ (self, data):
        super(aiCloud, self).__init__(data)
        self.CreateFromArray(self._data)
        self._data = aiArray(self.points_npy, ai_obj=self)

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

class aiCube (aiObject):
    def __init__ (self, scale):
        """Create a box with given xscale, yscale, zscale.

        Properties:
        center = [0, 0, 0]
        width, length, height = [1, 1, 1]
        bounds: xmin/xmax, ymin/ymax, zmin/zmax = [-0.5, 0.5]

        """
        cube_pts = np.array([[-0.5, -0.5, -0.5],
                             [ 0.5, -0.5, -0.5],
                             [-0.5,  0.5, -0.5],
                             [ 0.5,  0.5, -0.5],
                             [-0.5, -0.5,  0.5],
                             [ 0.5, -0.5,  0.5],
                             [-0.5,  0.5,  0.5],
                             [ 0.5,  0.5,  0.5]]).astype(np.float64)
        cube_pts[:, 0] *= scale[0]
        cube_pts[:, 1] *= scale[1]
        cube_pts[:, 2] *= scale[2]

        super(aiCube, self).__init__(cube_pts)
        self.CreateFromArray(cube_pts)

        faces = [(0,2,3,1), (4,6,7,5), (0,1,5,4), (2,3,7,6), \
                 (1,5,7,3), (0,4,6,2)]
        self.polys = mkVtkCells(faces)
        self.pd.SetPolys(self.polys)

        self._data = aiArray(self.points_npy, ai_obj=self)

        self.properties.LightingOff()
        self.wireframe = True

    @property
    def color (self):
        """ A box's color is defined by the actor, not the points """
        return self.actor_color
    @color.setter
    def color (self, color):
        self.actor_color = color

    @property
    def center (self):
        return np.mean(self._data, axis=0)

class aiPly (aiObject):
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

class aiAxis (aiObject):
    def __init__ (self, length=1):
        """Creates an axis object. This is useful for knowing the global frame of
        reference. (R, G, B) = (X, Y, Z) axis

        """
        super(aiAxis, self).__init__(np.array((0.,0.,0.)))
        self.CreateAxes(length)
        self.labels = False

    @property
    def labels (self):
        return bool(self.actor.GetAxisLabels())
    @labels.setter
    def labels (self, val):
        self.actor.SetAxisLabels(int(val))

class aiLine (aiObject):
    def __init__ (self, data):
        """ Creates a line object """
        assert (data.shape[0] == 2)
        super(aiLine, self).__init__(data)

        self.CreateFromArray(data)
        self._data = aiArray(self.points_npy, ai_obj=self)

        lines = [(0,1)]
        self.lines = mkVtkCells(lines)
        self.pd.SetLines(self.lines)

    @property
    def color (self):
        """ A line's color is defined by the actor, not the points """
        return self.actor_color
    @color.setter
    def color (self, color):
        self.actor_color = color

    @property
    def start (self):
        """ Get the first point in the line """
        return self.data[0, :]

    @property
    def end (self):
        """ Get the last point in the line """
        return self.data[-1, :]

class aiImage (aiObject):
    def __init__ (self, cv2_image):
        # We need to copy this image to keep a reference of it
        self.cv2_image = cv2_image.copy()
        # Vtk is RGB; openCV is BRG
        self.vtk_image = cv2.cvtColor(self.cv2_image, cv2.COLOR_BGR2RGB)
        super(aiImage, self).__init__(self.vtk_image)

        self.shape = self.cv2_image.shape
        self.width = self.cv2_image.shape[1]
        self.height = self.cv2_image.shape[0]

        # Read the image from the numpy array
        importer = vtk.vtkImageImport()
        importer.SetDataOrigin(-self.width/2, -self.height/2, 0)
        importer.SetWholeExtent(0, self.width - 1, 0, self.height - 1, 0, 0)
        importer.SetDataExtentToWholeExtent()
        importer.SetDataScalarTypeToUnsignedChar()
        importer.SetNumberOfScalarComponents(self.vtk_image.shape[2])
        importer.SetImportVoidPointer(self.vtk_image)

        flipY = vtk.vtkImageFlip()
        flipY.SetFilteredAxis(1)
        flipY.SetInputConnection(importer.GetOutputPort())
        flipY.Update()

        self.actor = vtk.vtkImageActor()
        self.actor.SetInput(flipY.GetOutput())

    def fillRenderer (self):
        """Fills the renderer with the image. This is just an approximation that works
        well for 1280x800 images

        """
        self.ren.ren.ResetCamera()
        self.ren.cam.Dolly(2)
        self.ren.cam.SetClippingRange(100, 100000)

    def idxToCoords (self, idx):
        """ Converts a picked index to the position in pixel space """
        if idx == -1:
            return (-1, -1)
        return (int(idx % self.width), int(self.height - idx / self.width))
