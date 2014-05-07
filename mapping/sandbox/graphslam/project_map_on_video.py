import os
import argparse
import cv2
import numpy as np
import h5py
from Q50_config import LoadParameters
from VideoReader import VideoReader
from chunk_and_align import get_enu0
from LidarTransforms import R_to_c_from_l
from graphslam_config import REALIGN_EVERY
from pipeline_config import EXPORT_STEP
from chunk_and_align import get_closest_key_value

# FIXME PARAM
WINDOW = 50*2.5

def cloudToPixels(cam, pts_wrt_cam):

    width = 4
    (pix, J) = cv2.projectPoints(pts_wrt_cam.transpose(), np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]), cam['KK'], cam['distort'])

    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0, :] > 0 + width/2)
    mask = np.logical_and(mask, pix[1, :] > 0 + width/2)
    # PARAM
    mask = np.logical_and(mask, pix[0, :] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1, :] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2, :] > 0)
    dist_sqr = np.sum(pts_wrt_cam[0:3, :] ** 2, axis=0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

def localMapToPixels(all_data, imu_transforms_t, T_from_i_to_l, cam):
    # load nearby map frames
    pts_wrt_imu_0 = np.array(all_data[:, 0:3]).transpose()
    pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0,
        np.ones((1, pts_wrt_imu_0.shape[1]))))
    # transform points from imu_0 to imu_t
    pts_wrt_imu_t = np.dot(np.linalg.inv(imu_transforms_t), pts_wrt_imu_0)
    # transform points from imu_t to lidar_t
    pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t)
    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = np.dot(R_to_c_from_l(cam),
            pts_wrt_camera_t.transpose())
    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    return (pix, mask)


if __name__ == '__main__':
    # Then add video generation w/ the matches to the pipeline
    parser = argparse.ArgumentParser(description='project lidar on video')
    parser.add_argument('video', help='video file')
    parser.add_argument('ldr1', help='npz file with lidar points, run 1')
    parser.add_argument('ldr2', help='npz file with lidar points, run 2')
    parser.add_argument('gps1', help='gps file 1')
    parser.add_argument('gps2', help='gps file 2')
    parser.add_argument('match_file', help='h5 file w/ matches')
    parser.add_argument('t1', help='imu transforms, run 1')
    # TODO determine whether we should even be using this
    parser.add_argument('t2', help='imu transforms, run 2')
    parser.add_argument('tb', help='file with transform between runs')
    parser.add_argument('outvideo', help='output video file')
    parser.add_argument('--cam', default=2, help='camera number to use')
    args = parser.parse_args()

    # Set up video and shared parameters

    cam_num = args.cam
    video_file = args.video
    # PARAM FIXME
    params = LoadParameters('q50_4_3_14_params')
    cam = params['cam'][cam_num-1]
    video_reader = VideoReader(video_file)
    T_from_i_to_l = np.linalg.inv(params['lidar']['T_from_l_to_i'])

    # Read in lidar data and transforms

    print args.gps1, args.gps2
    enu1 = get_enu0(args.gps1, args.gps1)
    enu2 = get_enu0(args.gps2, args.gps1)

    print args.t1, args.t2
    imu_transforms1 = np.load(args.t1)['data']
    imu_transforms2 = np.load(args.t2)['data']

    print args.ldr1, args.ldr2
    all_data1 = np.load(args.ldr1)['data']
    all_data2 = np.load(args.ldr2)['data']

    h5f = h5py.File(args.match_file, 'r')
    nn_matches = h5f['matches'][...]
    h5f.close()
    start2, start1 = nn_matches[0, :]
    start1 = start1 / EXPORT_STEP
    start2 = start2 / EXPORT_STEP
    nn_dict = dict()
    for t in range(nn_matches.shape[0]):
        nn_dict[nn_matches[t, 1]] = nn_matches[t, 0]

    tbs = list()
    chunk_num = 0
    for k in range(start1, start1 + nn_matches.shape[0] / EXPORT_STEP, REALIGN_EVERY):
        f = os.path.splitext(args.tb)[0] + '--%d' % chunk_num + '.h5'
        h5f = h5py.File(f, 'r')
        tb = h5f['transform'][...]
        tbs.append(tb)
        chunk_num += 1
    h5f.close()

    # Apply transform in global coordinates

    all_data2[:, 0:3] = all_data2[:, 0:3] + (enu2 - enu1)

    # Transforms computed by scan matching

    chunk_num = 0
    #all_data2[:, 0:3] = all_data2[:, 0:3] + tbs[0][0:3, 3]
    for k in range(start1, start1 + nn_matches.shape[0] / EXPORT_STEP, REALIGN_EVERY):
        start_ind = k * EXPORT_STEP
        t_start = get_closest_key_value(k * EXPORT_STEP, nn_dict, max_shift=10)
        t_final = get_closest_key_value(k * EXPORT_STEP + REALIGN_EVERY * EXPORT_STEP, nn_dict, max_shift=10)
        mask_window = (all_data2[:, 4] <= t_final) & (all_data2[:, 4] > t_start)
        all_data2[mask_window, 0:3] = all_data2[mask_window, 0:3] + tbs[chunk_num][0:3, 3]
        chunk_num += 1

    # BGR

    red = [0, 0, 255]
    green = [0, 255, 0]

    # Set up video writer

    fps = 30  # PARAM
    fourcc = cv2.cv.CV_FOURCC(*'MJPG')
    video_writer = cv2.VideoWriter()
    video_writer.open(args.outvideo, fourcc, fps, (1280, 960))  # PARAM

    # Read in images from video and project

    # FIXME PARAM
    t = 0
    while t < min(start1, start2) + 5000:
        for count in range(2):
            (success, I) = video_reader.getNextFrame()

        if not success:
            break

        t = video_reader.framenum - 1

        if t not in nn_dict or (t + WINDOW) not in nn_dict:
            continue

        print t

        mask_window = (all_data1[:, 4] < t + WINDOW) & (all_data1[:, 4] > t)
        all_data1_copy = np.array(all_data1[mask_window, :])

        mask_window = (all_data2[:, 4] < nn_dict[t + WINDOW]) & (all_data2[:, 4] > nn_dict[t])
        all_data2_copy = np.array(all_data2[mask_window, :])
        #print all_data1_copy.shape, all_data2_copy.shape

        # Reproject

        (pix1, mask1) = localMapToPixels(all_data1_copy, imu_transforms1[t, :, :], T_from_i_to_l, cam)
        #(pix2, mask2) = localMapToPixels(all_data2_copy, imu_transforms2[nn_dict[t], :, :], T_from_i_to_l, cam)
        (pix2, mask2) = localMapToPixels(all_data2_copy, imu_transforms1[t, :, :], T_from_i_to_l, cam)

        # Draw

        intensity1 = all_data1_copy[mask1, 3]
        intensity2 = all_data2_copy[mask2, 3]
        heat_colors1 = np.tile(red, (intensity1.shape[0], 1))
        heat_colors2 = np.tile(green, (intensity2.shape[0], 1))

        for p in range(2):
            I[pix1[1, mask1]+p, pix1[0, mask1], :] = heat_colors1
            I[pix1[1, mask1], pix1[0, mask1]+p, :] = heat_colors1
            I[pix1[1, mask1]+p, pix1[0, mask1], :] = heat_colors1
            I[pix1[1, mask1], pix1[0, mask1]+p, :] = heat_colors1

            I[pix2[1, mask2]+p, pix2[0, mask2], :] = heat_colors2
            I[pix2[1, mask2], pix2[0, mask2]+p, :] = heat_colors2
            I[pix2[1, mask2]+p, pix2[0, mask2], :] = heat_colors2
            I[pix2[1, mask2], pix2[0, mask2]+p, :] = heat_colors2

        video_writer.write(I)

    print 'Played %d frames' % t
    video_writer.release()
