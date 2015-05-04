import sys
import numpy as np

sys.path.append('../process/')
from transformations import euler_matrix
from aivtk import *

def update (renderers):
    cloud = renderers.cloud_ren.objects.clouds[0]
    cube = renderers.cloud_ren.objects.cubes[0]
    # We can update the position of clouds and cubes by accessing their data
    # directly
    cloud.data[10:20, :] += np.array([.001] * 3)
    # cube.data += np.array([.001] * 3)

if __name__ == '__main__':
    world = aiWorld((1600, 800))
    # Use the default framerate (60 hz)
    world.update_cb = update

    # ex: Use a 30Hz framerate
    # world.update_cb = (update, 30)

    cloud_ren = aiRenderer()
    # cloud_ren.ren.SetInteractive(False)

    def custom_left_press (x, y, ai_obj, idx, ren, default):
        if ai_obj in ren.objects.cubes:
            # Add some metadata to the renderer
            ren.meta.cube = ai_obj
            ren.meta.idx = idx
        else:
            default()
    def custom_left_release (x, y, ai_obj, idx, ren, default):
        if 'cube' in ren.meta:
            # When we release the mouse, delete the metadata
            del ren.meta.cube
            del ren.meta.idx
        else:
            default()
    def custom_mouse_move (x, y, ai_obj, idx, ren, default):
        if 'cube' in ren.meta:
            cube = ren.meta.cube
            cube_idx = ren.meta.idx
            # Unfortunately this does not work
            # cube.data[0,0] += 0.5
            # This will work
            # cube.data[0,0] += 0.5
            # cube.modified() # This will ensure the data changes
            # This way also works
            # cube.data[0, 0:1] += 0.5
            P = ren.displayToWorld(x, y, *cube.data[cube_idx, :])
            cube.data[cube_idx, :] = P
            cube.modified()
        else:
            default()

    cloud_ren.mouse_handler.leftPress = custom_left_press
    cloud_ren.mouse_handler.leftRelease = custom_left_release
    cloud_ren.mouse_handler.mouseMove = custom_mouse_move

    def custom_char_entered (key, ctrl, alt, ren, default):
        print ctrl, alt, key
        default()
    cloud_ren.key_handler.charEntered = custom_char_entered

    world.addRenderer(cloud_ren = cloud_ren)

    clouds = []
    num_pts = 100
    for i in xrange(3):
        pts = np.random.rand(num_pts, 3)*2 - 1
        cloud = aiCloud(pts)
        colors = (np.random.rand(*pts.shape) * 255).astype(np.uint8)
        cloud.color = colors
        cloud.pickable = False
        clouds.append(cloud)

    cube = aiCube([1,1,1])
    cube.color = [0, 0, 255]
    cube.point_size = 10
    # car = aiPly('gtr.ply')

    axis = aiAxis()
    axis.labels = True

    cloud_ren.addObjects(cubes=cube, clouds=clouds, axis=axis)
    # We can add objects to the renderer later
    # cloud_ren.addObjects(car = car)

    # We can add and delete objects as lists
    # cloud_ren.removeObjects(cloud_ren.objects.clouds[:2])

    # Access objects through their renderer by the name given in addObjects
    cloud_ren.objects.cubes[0].opacity = 1
    cloud_ren.objects.cubes[0].wireframe = True

    # Make sure to start the world (runs the update function)
    world.start()
