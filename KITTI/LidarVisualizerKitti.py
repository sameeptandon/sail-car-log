import sys
import numpy as np
sys.path.append('../process/')
sys.path.append('../visualization/')
from transformations import euler_matrix
from aivtk import *
from icp import icp
from LidarTransformsKitti import *
from ColorMap import heatColorMapFast
from MapBuilderKitti import MapBuilder
from ArgParserKitti import parse_args
from sklearn.neighbors import NearestNeighbors
from GPSReaderKitti import GPSReader
from GPSTransforms import *
previous_gps_data = None
previous_data = None
previous_time = None
def update (renderers):
    global previous_gps_data, previous_data, previous_time

    cloud_ren = renderers.cloud_ren
    if 'clouds' in cloud_ren.objects:
        cloud_ren.removeObjects(cloud_ren.objects.clouds)
    if 'clouds2' in cloud_ren.objects:
        cloud_ren.removeObjects(cloud_ren.objects.clouds2)
    # Load data

    # Above data contains point cloud of all points above car with height > 1.5 meters above lidar
    # (x, y, z) correspond to forward, left, up coordinates
    above_data, t_data = renderers.cloud_ren.meta.mb.getCurrentData('above',local=True)
    # Planar data contains point cloud of all points on road
    #planar_data, t_planar_data = renderers.cloud_ren.meta.mb.getCurrentData('lanes',local=True)

    #data = np.concatenate((above_data,planar_data), axis=0)
    data = above_data
    #current_gps_data, t_planar_data = renderers.cloud_ren.meta.mb.getCurrentData('gps',local=True)
    

    n_iter=10
    threshold = 0.05
    #inliers = renderers.cloud_ren.meta.pf.getPlanarPoints(planar_data[:,:3], n_iter,threshold)
    #planar_data = planar_data[inliers,:]

    newclouds = []
    cloud = aiCloud(data[:,:3])
    color = np.zeros(data[:,0:3].shape, dtype='u1')
    color[:,2] = 255
    cloud.color = color
    #colors = np.squeeze(heatColorMapFast(data[:,3], 0,255))
    #cloud.color = (colors[:,::-1]).astype('u1', copy=False)
    newclouds.append(cloud)
    cloud_ren.addObjects(clouds=newclouds)

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
      #t = relative_trans[0:3,3:4]
      print "init_R: " +str(relative_trans[0:3,0:3])
      print "R: " + str(R)
      print "init_t: " +str(relative_trans[0:3,3])
      print "t: " + str(t)
      previous_data[:,:3] = (np.dot(R,previous_data[:,:3].T) + np.tile(t, (1, len(previous_data)))).T

      newclouds2 = []
      cloud2 = aiCloud(previous_data[:,:3])
      color2 = np.zeros(previous_data[:,0:3].shape, dtype='u1')
      color2[:,0] = 255
      cloud2.color = color2
      #colors = np.squeeze(heatColorMapFast(data[:,3], 0,255))
      #cloud.color = (colors[:,::-1]).astype('u1', copy=False)
      newclouds2.append(cloud2)
      cloud_ren.addObjects(clouds2=newclouds2)
    renderers.cloud_ren.meta.mb.stepForward()
    previous_data = data.copy()
    previous_time = int(current_time)

    

def __get_rmse(previous_gps_data_points, current_gps_data_points, R, t):
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(current_gps_data_points)
    distances, indices = nbrs.kneighbors(previous_gps_data_points)
    current_gps_data_points_actual = current_gps_data_points[indices.T][0]
    current_gps_data_points_estimate = np.dot(R,previous_gps_data_points.T) + np.tile(t, (1, len(previous_gps_data_points)))
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

    # load a 3d car model
    car = aiPly('../mapping/viz/gtr.ply')
    # put the car in the center
    initCarTrans = np.eye(4)
    initCarTrans[:,0]=np.array([0,1,0,0])
    initCarTrans[:,1]=np.array([-1,0,0,0])
    initCarTrans[:,3]=np.array([3.2,-1.85,-2,1])
    car.actor.SetUserTransform(array2vtkTransform(initCarTrans))
    cloud_ren.meta.initCarTrans = initCarTrans

    cloud_ren.meta.mb = MapBuilder(args, 1, 600, 0.1, 0.1,absolute=True)
    plane_file = args['fullname'] + '_ground.npz'
    lanes_file = sys.argv[1] + '/multilane_points.npz'
    #cloud_ren.meta.pf = PlaneFitter(args, plane_file, lanes_file)
    cloud_ren.addObjects(axis=axis)
    cloud_ren.addObjects(car=car)

    gps_reader = GPSReader(args['gps_mark1'])
    gps_data = gps_reader.getNumericData()
    cloud_ren.meta.imu_transforms_mk1 = absoluteTransforms(gps_data)
    cloud_ren.meta.gps_times_mk1 = gps_data[:,0]*1e6
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
