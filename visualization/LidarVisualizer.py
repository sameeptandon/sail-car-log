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
from sklearn.neighbors import NearestNeighbors

previous_gps_data = None
previous_data = None

def update (renderers):
    global previous_gps_data, previous_data

    cloud_ren = renderers.cloud_ren
    if 'clouds' in cloud_ren.objects:
        cloud_ren.removeObjects(cloud_ren.objects.clouds)
    # Load data

    # Above data contains point cloud of all points above car with height > 1.5 meters above lidar
    # (x, y, z) correspond to forward, left, up coordinates
    above_data, t_data = renderers.cloud_ren.meta.mb.getCurrentData('above',local=True)
    # Planar data contains point cloud of all points on road
    planar_data, t_planar_data = renderers.cloud_ren.meta.mb.getCurrentData('lanes',local=True)

    data = np.concatenate((above_data,planar_data), axis=0)
    current_gps_data, t_planar_data = renderers.cloud_ren.meta.mb.getCurrentData('gps',local=True)
    renderers.cloud_ren.meta.mb.stepForward()

    if previous_data is not None:
        previous_data_points = previous_data[:, :3]
        data_points = data[:, :3]
        R, t = icp(previous_data_points, data_points)
        #print "R: " + str(R)
        print "t: " + str(t)
        #print "RMSE: " + str(__get_rmse(previous_gps_data_points, current_gps_data_points, R, t))
    previous_data = data.copy()

    n_iter=10
    threshold = 0.05
    #inliers = renderers.cloud_ren.meta.pf.getPlanarPoints(planar_data[:,:3], n_iter,threshold)
    #planar_data = planar_data[inliers,:]
    data = np.concatenate((above_data,planar_data), axis=0)
    newclouds = []
    cloud = aiCloud(data[:,:3])
    colors = np.squeeze(heatColorMapFast(data[:,3], 0,255))
    cloud.color = (colors[:,::-1]).astype('u1', copy=False)
    newclouds.append(cloud)
    cloud_ren.addObjects(clouds=newclouds)

def __get_rmse(previous_gps_data_points, current_gps_data_points, R, t):
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(current_gps_data_points)
    distances, indices = nbrs.kneighbors(previous_gps_data_points)
    current_gps_data_points_actual = current_gps_data_points[indices.T][0]
    current_gps_data_points_estimate = R*previous_gps_data_points.T + np.tile(t, (1, len(previous_gps_data_points)))
    current_gps_data_points_estimate = current_gps_data_points_estimate.T
    err = current_gps_data_points_actual - current_gps_data_points_estimate
    err = np.multiply(err, err)
    err = np.sum(err)
    rmse = np.sqrt(err/len(previous_gps_data_points))
    return rmse

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
    cloud_ren.meta.mb = MapBuilder(args, 64, 600, 0.1, 0.1,absolute=True)
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
