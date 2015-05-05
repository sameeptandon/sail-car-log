import sys
import numpy as np
import cv2

sys.path.append('../process/')
from transformations import euler_matrix
from VideoReader import VideoReader

from aivtk import *

def update (renderers):
    cloud = renderers.cloud_ren.objects.clouds[0]
    # handle = renderers.cloud_ren.objects.handles[0]
    # We can update the position of clouds and cubes by accessing their data
    # directly
    # cloud.data[10:20, :] += np.array([.001] * 3)
    # handle.data += np.array([.001] * 3)
    # cube.data += np.array([.001] * 3)

def custom_left_press (x, y, ai_obj, idx, ren, default):
    if ai_obj in ren.objects.handles and idx == 1:
        # Add some metadata to the renderer
        ren.meta.handle = ai_obj
        ren.meta.idx = idx

        cube = ai_obj.group[1]
        ren.cam_focal_point = cube.center
    else:
        default()

def custom_left_release (x, y, ai_obj, idx, ren, default):
    if 'handle' in ren.meta:
        # When we release the mouse, delete the metadata
        del ren.meta.handle
        del ren.meta.idx
    else:
        default()

def custom_mouse_move (x, y, ai_obj, idx, ren, default):
    if 'handle' in ren.meta:
        handle = ren.meta.handle
        idx = ren.meta.idx

        P = ren.displayToWorld(x, y, *handle.data[idx, :])

        vec = P - handle.start
        rad = np.arctan2(vec[1], vec[0])
        xform = euler_matrix(0, 0, rad)
        xform[:, 3] = handle.transform[:, 3]

        for obj in handle.group:
            obj.transform = np.linalg.inv(obj.transform)
            obj.transform = xform
    else:
        default()

def custom_char_entered (key, ctrl, alt, ren, default):
    print ctrl, alt, key
    if key == 'q':
        ren.world.quit()

if __name__ == '__main__':
    world = aiWorld((1280, 640))
    # Use the default framerate (60 hz)
    world.update_cb = update
    # Use a 30Hz framerate
    # world.update_cb = (update, 30)

    cloud_ren = aiRenderer()
    img_ren = aiRenderer(*(.5, .5, 1, 1))
    # img_ren.interactive = False

    world.addRenderer(cloud_ren = cloud_ren, img_ren = img_ren)

    cloud_ren.mouse_handler.leftPress = custom_left_press
    cloud_ren.mouse_handler.leftRelease = custom_left_release
    cloud_ren.mouse_handler.mouseMove = custom_mouse_move

    cloud_ren.key_handler.charEntered = custom_char_entered

    clouds = []
    num_pts = 1000
    for i in xrange(3):
        pts = np.random.rand(num_pts, 3)*20 - 10
        cloud = aiCloud(pts)
        colors = (np.random.rand(*pts.shape) * 255).astype(np.uint8)
        cloud.color = colors
        cloud.pickable = False
        clouds.append(cloud)

    cubes = []
    handles = []
    for i in xrange(20):
        rot = np.random.rand(3) * np.pi
        rot[0:2] = 0

        xform = euler_matrix(*tuple(rot))
        xform[:3, 3] = np.random.rand(3) * 20 - 10

        cube = aiCube([1,1,1])
        cube.color = np.random.rand(3) * 255
        cube.transform = xform

        handle = aiLine(np.array([[0.,0.,0.], [1.,0.,0.]]))
        handle.point_size = 10
        handle.transform = xform

        # This links the objects together (see aiObject.group)
        handle.link(cube)

        cubes.append(cube)
        handles.append(handle)

    axis = aiAxis()
    # axis.labels = True

    # car = aiPly('assets/gtr.ply')

    cloud_ren.addObjects(cubes=cubes, handles=handles, clouds=clouds, axis=axis)

    # We can add objects to the renderer later
    # cloud_ren.addObjects(car = car)

    # We can add and delete objects as lists
    # cloud_ren.removeObjects(cloud_ren.objects.clouds[:2])

    # Access objects through their renderer by the name given in addObjects
    cloud_ren.objects.cubes[0].opacity = 1
    cloud_ren.objects.cubes[0].wireframe = True

    # Set up a top down view
    cloud_ren.cam_position = (0, 0, 20)
    cloud_ren.cam_view_up = (0, 1, 0)

    # Load the image from a file
    # reader = # VideoReader('../process/data/4-17-15-280-test/101n_b/101n_b601.zip')
    # (success, I) = reader.getNextFrame()
    I = cv2.imread('assets/placeholder.jpg')
    img = aiImage(I)
    img_ren.addObjects(img=img)

    # Have the image fill the renderer
    img.fillRenderer()

    # Make sure to start the world (runs the update function)
    world.start()
