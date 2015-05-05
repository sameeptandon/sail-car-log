import sys
import numpy as np

sys.path.append('../process/')
from transformations import euler_matrix
from aivtk import *
from LidarTransforms import *
from ColorMap import *

global fileidx
global ldr_files

def update (renderers):
    global fileidx
    global ldr_files
    cloud = renderers.cloud_ren.objects.clouds[0]
    ldr_data = loadLDR(ldr_files[fileidx])
    print ldr_data.shape
    cloud.data = np.ascontiguousarray(ldr_data[:,:3])
    colors = np.squeeze(heatColorMapFast(ldr_data[:,4], 0,255))
    cloud.color = colors[:,::-1]
    fileidx+=1

    # handle = renderers.cloud_ren.objects.handles[0]
    # We can update the position of clouds and cubes by accessing their data
    # directly
    # cloud.data[10:20, :] += np.array([.001] * 3)
    # handle.data += np.array([.001] * 3)
    # cube.data += np.array([.001] * 3)

if __name__ == '__main__':
    fileidx = 0
    ldr_dir = '../process/data/4-25-15-elcamino/el-camino_b/el-camino_b_frames'
    ldr_files = sorted(list(glob.glob(os.path.join(ldr_dir, '*.ldr'))))
    world = aiWorld((1280, 640))
    # Use the default framerate (60 hz)
    world.update_cb = update
    # Use a 30Hz framerate
    # world.update_cb = (update, 30)

    cloud_ren = aiRenderer()
    # cloud_ren.ren.SetInteractive(False)

    def custom_left_press (x, y, ai_obj, idx, ren, default):
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

    cloud_ren.mouse_handler.leftPress = custom_left_press
    cloud_ren.mouse_handler.leftRelease = custom_left_release
    cloud_ren.mouse_handler.mouseMove = custom_mouse_move

    def custom_char_entered (key, ctrl, alt, ren, default):
        print ctrl, alt, key
        default()
    cloud_ren.key_handler.charEntered = custom_char_entered

    world.addRenderer(cloud_ren = cloud_ren)

    clouds = []


    ldr_data = loadLDR(ldr_files[0])
    cloud = aiCloud(ldr_data[:,:3])
    colors = np.squeeze(heatColorMapFast(ldr_data[:,4], 0,255))
    cloud.color = colors[:,::-1]
    cloud.pickable = False
    clouds.append(cloud)


    axis = aiAxis()
    # axis.labels = True

    # car = aiPly('gtr.ply')
    pts = loadLDR(ldr_files[0])
    cloud_ren.addObjects(clouds=clouds, axis=axis)

    # We can add objects to the renderer later
    # cloud_ren.addObjects(car = car)

    # We can add and delete objects as lists
    # cloud_ren.removeObjects(cloud_ren.objects.clouds[:2])


    # Set up a top down view
    cloud_ren.cam_position = (0, 0, 20)
    cloud_ren.cam_view_up = (0, 1, 0)

    # Make sure to start the world (runs the update function)
    world.start()
