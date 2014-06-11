from Q50_config import *
import sys, os
from GPSReader import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
import numpy as np
import cv2
from ArgParser import *
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
from matplotlib.cm import jet, rainbow
from LidarIntegrator import start_fn

WINDOW = 2#50*2.5

#IMG_WIDTH = 1280
#IMG_HEIGHT = 960
IMG_WIDTH = 2080
IMG_HEIGHT = 1552

def cloudToPixels(cam, pts_wrt_cam): 

    width = 4
    (pix, J)  = cv2.projectPoints(pts_wrt_cam.transpose(), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), cam['KK'], cam['distort'])

    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 2080 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 1552 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

def localMapToPixels(map_data, imu_transforms_t, T_from_i_to_l, cam):
    # load nearby map frames
    pts_wrt_imu_0 = array(map_data[:,0:3]).transpose()
    pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0, 
        np.ones((1,pts_wrt_imu_0.shape[1]))))
    # transform points from imu_0 to imu_t
    pts_wrt_imu_t = np.dot( np.linalg.inv(imu_transforms_t), pts_wrt_imu_0)
    # transform points from imu_t to lidar_t
    pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t);
    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = dot(R_to_c_from_l(cam), 
            pts_wrt_camera_t.transpose())

    pts_wrt_camera_t = np.vstack((pts_wrt_camera_t,
        np.ones((1,pts_wrt_camera_t.shape[1]))))
    pts_wrt_camera_t = dot(cam['E'], pts_wrt_camera_t)
    pts_wrt_camera_t = pts_wrt_camera_t[0:3,:]

    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    return (pix, mask, pts_wrt_imu_t)


def trackbarOnchange(t, prev_t):
    if abs(t - prev_t) > 1:
        video_reader.setFrame(t)


if __name__ == '__main__':

    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    video_file = args['video']

    params = args['params'] 
    cam = params['cam'][cam_num-1]
    video_reader = VideoReader(video_file)
    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    
    T_from_i_to_l = np.linalg.inv(params['lidar']['T_from_l_to_i'])

    all_data = np.load(sys.argv[3])
    map_data = all_data['data']
    #map_data = map_data[map_data[:,3] > 60, :]
    # map points are defined w.r.t the IMU position at time 0
    # each entry in map_data is (x,y,z,intensity,framenum). 

    print "Hit 'q' to quit"
    trackbarInit = False

    interp_grid = np.mgrid[0:IMG_WIDTH, 0:IMG_HEIGHT]

    #pix_list = list()
    #depth_list = list()
    #img_list = list()

    fps = 10  # PARAM
    fourcc = cv2.cv.CV_FOURCC(*'MJPG')
    video_writer = cv2.VideoWriter()
    video_writer.open('tmp.avi', fourcc, fps, (IMG_WIDTH, IMG_HEIGHT))  # PARAM

    video_reader.setFrame(start_fn)

    #while len(pix_list) < 10:
    while True:
        for count in range(10):
            (success, I) = video_reader.getNextFrame()
            #print I.shape

        if not success:
            print 'Done reading video', video_file
            break

        t = video_reader.framenum - 1
        print t
        mask_window = (map_data[:,4] < t + WINDOW) & (map_data[:,4] > t )
        map_data_copy = array(map_data[mask_window, :])
        if map_data_copy.size == 0:
            print 'Map data empty'
            break

        # reproject
        (pix, mask, pts_wrt_imu_t) = localMapToPixels(map_data_copy, imu_transforms[t,:,:], T_from_i_to_l, cam)

        # draw
        pix = pix[:, mask]
        intensity = map_data_copy[mask, 3]
        depth = pts_wrt_imu_t[0, mask]

        img_interp = griddata(pix.T, depth, interp_grid.T)
        img_interp[np.isnan(img_interp)] = max(depth)  # PARAM
        print max(depth)

        img_color = rainbow(img_interp / max(depth))  # PARAM
        #plt.imshow(img_color)
        #plt.show()
        img_color = img_color[:, :, 0:3]
        img_color = np.uint8(img_color * 255)

        img_out = np.uint8(0.0 * I + 1.0*img_color)
        video_writer.write(img_out)

        #pix_list.append(pix[:, mask])
        #depth_list.append(depth)
        #img_list.append(I)

        #heat_colors = heatColorMapFast(depth, 0, 100)
        #for p in range(4):
            #I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            #I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]

        #cv2.imshow(video_file, cv2.pyrDown(I))
        #if not trackbarInit:
            #cv2.createTrackbar('trackbar', video_file, 0, int(video_reader.total_frame_count), lambda x: trackbarOnchange(x, t))
            #trackbarInit = True
        #else:
            #cv2.setTrackbarPos('trackbar', video_file, t)

        #keycode = cv2.waitKey(1)
        #if keycode == 113:
            #break

    video_writer.release()

    print 'Played %d frames' % t

    '''
    # Save pickled data
    import pickle
    data = {
        'pix_list': pix_list,
        'depth_list': depth_list,
        'img_list': img_list
    }
    pickle.dump(data, open('pix_depth.pkl', 'wb'))
    '''
