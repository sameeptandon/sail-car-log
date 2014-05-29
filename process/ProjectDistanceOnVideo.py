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
import h5py

WINDOW = 250*2.5

def cloudToPixels(cam, pts_wrt_cam): 

    width = 4
    (pix, J)  = cv2.projectPoints(pts_wrt_cam.transpose(), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), cam['KK'], cam['distort'])

    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
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

    return (pix, mask)


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

    # Left and right data
    left_file = np.load(sys.argv[3])
    left_data = left_file['data']
    right_file = np.load(sys.argv[4])
    right_data = right_file['data']

    # Label left and right data
    left_data = np.hstack((left_data, np.zeros((left_data.shape[0], 1))))
    right_data = np.hstack((right_data, np.ones((right_data.shape[0], 1))))

    map_data = np.vstack((left_data, right_data))
    print map_data.shape

    #map_data = map_data[map_data[:,3] > 60, :]
    # map points are defined w.r.t the IMU position at time 0
    # each entry in map_data is (x,y,z,intensity,framenum). 

    red = [0, 0, 255]
    blue = [255, 0, 0]
    green = [0, 255, 0]

    fps = 30  # PARAM
    fourcc = cv2.cv.CV_FOURCC(*'MJPG')
    video_writer = cv2.VideoWriter()
    video_writer.open('tmp.avi', fourcc, fps, (1280, 960))  # PARAM

    print "Hit 'q' to quit"
    trackbarInit = False
    init = False
    num_skipped = 0

    pts_3d_left = list()
    pts_3d_right = list()
    pix_2d_left = list()
    pix_2d_right = list()
    ts = list()

    while True:
        if not init:
            (success, I) = video_reader.getNextFrame()
            init = True
        else:
            for count in range(10):
                (success, I) = video_reader.getNextFrame()

        if not success:
            break

        t = video_reader.framenum - 1
        print t
        mask_window = (map_data[:,4] < t + WINDOW) & (map_data[:,4] > t );
        map_data_copy = array(map_data[mask_window, :]);

        pts_wrt_imu_0 = np.vstack((map_data_copy[:,0:3].transpose(),
            np.ones((1, map_data_copy.shape[0]))))
        transformed_map_data = np.dot(np.linalg.inv(imu_transforms[t, :, :]), pts_wrt_imu_0)

        far_mask = (np.sqrt(np.sum(np.square(transformed_map_data[0:3,:]), axis=0)) > 165) & (np.sqrt(np.sum(np.square(transformed_map_data[0:3,:]), axis=0)) < 175)
        far_map_data = map_data_copy[far_mask, :]
        close_map_data = map_data_copy[np.logical_not(far_mask), :]

        far_map_data_left = far_map_data[far_map_data[:, 5] == 0, :]
        far_map_data_right = far_map_data[far_map_data[:, 5] == 1, :]
        #print far_map_data_left.shape
        #print far_map_data_right.shape

        # reproject
        #(close_pix, close_mask) = localMapToPixels(close_map_data, imu_transforms[t,:,:], T_from_i_to_l, cam)
        #print far_map_data_right.shape
        if far_map_data_left.shape[0] == 0:
            print 'skipping left', t
            num_skipped += 1
            continue
        if far_map_data_right.shape[0] == 0:
            print 'skipping right', t
            num_skipped += 1
            continue

        far_map_data_left = np.median(far_map_data_left[:, 0:3], axis=0).reshape((1, 3))
        far_map_data_right = np.median(far_map_data_right[:, 0:3], axis=0).reshape((1, 3))

        (far_pix_left, far_mask_left) = localMapToPixels(far_map_data_left, imu_transforms[t,:,:], T_from_i_to_l, cam)
        (far_pix_right, far_mask_right) = localMapToPixels(far_map_data_right, imu_transforms[t,:,:], T_from_i_to_l, cam)

        # Find the closest far pix and get 3d and pixel information

        #closest_left = np.sqrt(np.sum(np.square(far_map_data_left[:, 0:3]), axis=1)).argmin()
        #closest_left = far_map_data_left[:, 4].argmax()
        #closest_right = far_map_data_right[:, 4].argmin()
        #closest_right = np.sum(np.square(far_map_data_right[:, 0:2] - far_map_data_left[closest_left, 0:2])).argmin()

        #pts_3d_left.append(far_map_data_left[closest_left, 0:3])
        #pts_3d_right.append(far_map_data_right[closest_right, 0:3])
        pts_3d_left.append(far_map_data_left.ravel())
        pts_3d_right.append(far_map_data_right.ravel())

        #print far_mask.shape
        #far_mask_left = np.array([far_mask_left[closest_left]])
        #far_mask_right = np.array([far_mask_right[closest_right]])
        #print far_mask.shape

        # draw 
        #intensity = map_data_copy[mask, 3]
        #heat_colors = heatColorMapFast(intensity, 0, 100)

        green_colors = np.tile(green, (np.sum(far_mask_left), 1))
        blue_colors = np.tile(blue, (np.sum(far_mask_right), 1))

        #pt_left = tuple(far_pix_left[:, far_mask_left].ravel())
        #pt_right = tuple(far_pix_right[:, far_mask_right].ravel())
        pt_left = tuple(far_pix_left.ravel())
        pt_right = tuple(far_pix_right.ravel())

        pix_2d_left.append(pt_left)
        pix_2d_right.append(pt_right)

        ts.append(t)

        cv2.circle(I, pt_left, 3, green, -1)
        cv2.circle(I, pt_right, 3, blue, -1)
        cv2.line(I, pt_left, pt_right, red, 2)

        #cv2.imshow(video_file, cv2.pyrDown(I))
        #if not trackbarInit:
            #cv2.createTrackbar('trackbar', video_file, 0, int(video_reader.total_frame_count), lambda x: trackbarOnchange(x, t))
            #trackbarInit = True
        #else:
            #cv2.setTrackbarPos('trackbar', video_file, t)

        #keycode = cv2.waitKey(1)
        #if keycode == 113:
            #break

        video_writer.write(I)

    print 'Played %d frames' % t
    print 'skipped', num_skipped
    video_writer.release()

    import IPython; IPython.embed()

    # Save data

    h5f = h5py.File('tmp.h5', 'w')
    h5f['pix_2d_left'] = np.vstack(pix_2d_left)
    h5f['pix_2d_right'] = np.vstack(pix_2d_right)
    h5f['pts_3d_left'] = np.vstack(pts_3d_left)
    h5f['pts_3d_right'] = np.vstack(pts_3d_right)
    h5f['ts'] = np.array(ts)
    h5f.close()
