import sys
import numpy as np
sys.path.append('../process/')
from transformations import euler_matrix
from aivtk import *
from icp import icp
from LidarTransforms import *
from ColorMap import heatColorMapFast
from MapBuilderICP import MapBuilder
from ArgParser import parse_args
from PlaneFitting import PlaneFitter

previous_gps_data = None

def update (renderers):
    global previous_gps_data

    cloud_ren = renderers.cloud_ren
    if 'clouds' in cloud_ren.objects:
        cloud_ren.removeObjects(cloud_ren.objects.clouds)
    # Load data

    # Above data contains point cloud of all points above car with height > 1.5 meters above lidar
    # (x, y, z) correspond to forward, left, up coordinates
    above_data, t__data = renderers.cloud_ren.meta.mb.getCurrentData('above',local=True)
    # Planar data contains point cloud of all points on road
    planar_data, t_planar_data = renderers.cloud_ren.meta.mb.getCurrentData('road',local=True)
    current_gps_data, t_planar_data = renderers.cloud_ren.meta.mb.getCurrentData('gps',local=True)
    if previous_gps_data != None:
        R, t = icp(previous_gps_data[:, :3], current_gps_data[:, :3])
        print "R: " + str(R)
        print "t: " + str(t)
    previous_gps_data = current_gps_data.copy()
    n_iter=10
    threshold = 0.05
    #inliers = renderers.cloud_ren.meta.pf.getPlanarPoints(planar_data[:,:3], n_iter,threshold)
    #planar_data = planar_data[inliers,:]
    data = np.concatenate((above_data,planar_data), axis=0)
    newclouds = []
    cloud = aiCloud(data[:,:3])
    colors = np.squeeze(heatColorMapFast(data[:,3], 0,255))
    cloud.color = (colors[:,::-1]).astype('u1')
    newclouds.append(cloud)
    cloud_ren.addObjects(clouds=newclouds)


if __name__ == '__main__':


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

    #cloud_ren.mouse_handler.leftPress = custom_left_press
    #cloud_ren.mouse_handler.leftRelease = custom_left_release
    #cloud_ren.mouse_handler.mouseMove = custom_mouse_move

    def custom_char_entered (key, ctrl, alt, ren, default):
        print ctrl, alt, key
        default()
    cloud_ren.key_handler.charEntered = custom_char_entered

    world.addRenderer(cloud_ren = cloud_ren)
    axis = aiAxis()

    args = parse_args(sys.argv[1], sys.argv[2])
    cloud_ren.meta.mb = MapBuilder(args, 1, 600, 0.1, 0.1,absolute=True)
    plane_file = args['fullname'] + '_ground.npz'
    lanes_file = sys.argv[1] + '/multilane_points.npz'
    cloud_ren.meta.pf = PlaneFitter(args, plane_file, lanes_file)
    cloud_ren.addObjects(axis=axis)

    #cloud_ren.meta.fileidx = 0
    #ldr_dir = '../process/data/4-25-15-elcamino/el-camino_b/el-camino_b_frames'
    #cloud_ren.meta.ldr_files = sorted(list(glob.glob(os.path.join(ldr_dir, '*.ldr'))))
    # We can add objects to the renderer later
    # cloud_ren.addObjects(car = car)

    # We can add and delete objects as lists
    # cloud_ren.removeObjects(cloud_ren.objects.clouds[:2])


    # Set up a top down view
    cloud_ren.cam_position = (0, 0, 20)
    cloud_ren.cam_view_up = (0, 1, 0)

    # Make sure to start the world (runs the update function)
    world.start()
