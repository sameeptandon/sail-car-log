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
from LaneCorrector import load_ply
from LaneMarkingHelper import get_transforms
previous_gps_data = None
previous_data = None
previous_time = None


def getCameraPosition(imu_transforms_mk1, t, focus=100):
    offset = np.array([-75.0, 0, 5.0]) / 4
    # Rotate the camera so it is behind the car
    position = np.dot(imu_transforms_mk1[t, 0:3, 0:3], offset)
    position += imu_transforms_mk1[t, 0:3, 3] + position

    # Focus 10 frames in front of the car
    focal_point = imu_transforms_mk1[t + focus, 0:3, 3]
    return position, focal_point




def update (renderers):
    global previous_gps_data, previous_data, previous_time

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
    #current_gps_data, t_planar_data = renderers.cloud_ren.meta.mb.getCurrentData('gps',local=True)
    current_time = renderers.cloud_ren.meta.mb.current_time
    if previous_data is not None and previous_time is not None:

        current_trans = get_transform_by_time(current_time, cloud_ren.meta.imu_transforms_mk1, cloud_ren.meta.gps_times_mk1)
        previous_trans = get_transform_by_time(previous_time, cloud_ren.meta.imu_transforms_mk1, cloud_ren.meta.gps_times_mk1)

        # transformation from previous frame to current frame
        relative_trans = np.dot(np.linalg.inv(previous_trans),current_trans)
        previous_data_points = previous_data[:, :3]
        data_points = data[:, :3]
        # initialize icp using R and t measured by the imu, and fine-tune from there.
        R, t = icp(previous_data_points, data_points, relative_trans[0:3,0:3], relative_trans[0:3,3:4])
        #print "R: " + str(R)
        #print "t: " + str(t)
        #print "RMSE: " + str(__get_rmse(previous_gps_data_points, current_gps_data_points, R, t))
        #n_iter=10
        #threshold = 0.05
    
        # use lidar icp to estimate translation between frames, instead of using the imu measurements.
        ll = np.dot(previous_trans[0:3, 0:3], t)
        current_trans[0:3, 3] = previous_trans[0:3, 3] + np.squeeze(ll) 
        # update car's position in map.
        current_car_trans = np.dot(current_trans,initCarTrans)
        cloud_ren.objects['car'][0].actor.SetUserTransform(array2vtkTransform(current_car_trans))
        # update camera position and focal point
        offset = np.array([0, 12, 16])
        # Rotate the camera so it is behind the car
        position = np.dot(current_car_trans[0:3, 0:3], offset)
        position += current_car_trans[0:3, 3] + position
        cloud_ren.cam_position = position    
        cloud_ren.cam_focal_point = current_car_trans[0:3, 3]
        cloud_ren.cam_view_up = (0, 0, 1)
    renderers.cloud_ren.meta.mb.stepForward()
    previous_data = data.copy()
    previous_time = int(current_time)
    


    #inliers = renderers.cloud_ren.meta.pf.getPlanarPoints(planar_data[:,:3], n_iter,threshold)
    #planar_data = planar_data[inliers,:]


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
    args = parse_args(sys.argv[1], sys.argv[2])

    world = aiWorld((1280, 640))
    # Use the default framerate (60 hz)
    world.update_cb = update
    # Use a 30Hz framerate
    # world.update_cb = (update, 30)

    cloud_ren = aiRenderer()
    world.addRenderer(cloud_ren = cloud_ren)
    # load raw map points generated using MapBuilder.
    raw_map_fname = args['fullname'] + '_no-cars.npz'
    raw_npz = np.load(raw_map_fname)
    raw_data = raw_npz['data']
    rawclouds = []
    cloud = aiCloud(raw_data[:,:3])
    colors = np.squeeze(heatColorMapFast(raw_data[:,3], 0,255))
    cloud.color = (colors[:,::-1]).astype('u1', copy=False)
    rawclouds.append(cloud)
    cloud_ren.addObjects(rawclouds=rawclouds)


    (imu_transforms_mk1, gps_data_mk1,gps_times_mk1) = get_transforms(args, 'mark1', absolute=True)
    cloud_ren.meta.imu_transforms_mk1 = imu_transforms_mk1
    cloud_ren.meta.gps_times_mk1 = gps_times_mk1



    axis = aiAxis()
    # load a 3d car model
    car = aiPly('../mapping/viz/gtr.ply')
    # put the car in the center
    initCarTrans = np.eye(4)
    initCarTrans[:,0]=np.array([0,1,0,0])
    initCarTrans[:,1]=np.array([-1,0,0,0])
    initCarTrans[:,3]=np.array([3.2,-1.85,-2,1])
    car.actor.SetUserTransform(array2vtkTransform(initCarTrans))
    cloud_ren.meta.initCarTrans = initCarTrans


    # car is stopped at 64s. So start from there
    cloud_ren.meta.mb = MapBuilder(args, 64, 600, 0.1, 0.1,absolute=True)
    #cloud_ren.meta.mb = MapBuilder(args, 1, 600, 0.1, 0.1,absolute=True)
    #plane_file = args['fullname'] + '_ground.npz'
    #lanes_file = sys.argv[1] + '/multilane_points.npz'
    #cloud_ren.meta.pf = PlaneFitter(args, plane_file, lanes_file)
    cloud_ren.addObjects(axis=axis)
    cloud_ren.addObjects(car=car)
    #cloud_ren.meta.fileidx = 0
    #ldr_dir = '../process/data/4-25-15-elcamino/el-camino_b/el-camino  _b_frames'
    #cloud_ren.meta.ldr_files = sorted(list(glob.glob(os.path.join(ldr_dir, '*.ldr'))))
    # We can add objects to the renderer later
    # cloud_ren.addObjects(car = car)

    # We can add and delete objects as lists
    # cloud_ren.removeObjects(cloud_ren.objects.clouds[:2])


    # Set up a top down view
    cloud_ren.cam_position = (0, 12, 16)
    cloud_ren.cam_view_up = (0, 0, 1)

    # Make sure to start the world (runs the update function)
    world.start()
