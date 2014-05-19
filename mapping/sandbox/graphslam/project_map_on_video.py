import h5py
import argparse
import cv2
import numpy as np
from Q50_config import LoadParameters
from VideoReader import VideoReader
from LidarTransforms import R_to_c_from_l
from pipeline_config import PROJECT_MAP_WINDOW
from graphslam_config import MAX_VIDEO_FRAMES


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


def draw_points(I, pix, mask, colors, rad=2):
    N = np.sum(mask)
    for p in range(rad):
        i0 = np.minimum(pix[1, mask]+p, I.shape[0] - 1)
        i1 = np.minimum(pix[0, mask]+p, I.shape[1] - 1)
        I[i0, pix[0, mask], :] = colors
        I[pix[1, mask], i1, :] = colors


def draw_hulls(I, cluster_labels, pxs, mask, color, thickness):
    pxs = pxs[:, mask]
    cluster_labels = cluster_labels[mask]
    for label in set(cluster_labels.tolist()):
        if label == -1:
            continue
        mask = cluster_labels == label
        masked_pxs = pxs[:, mask]
        hull_pxs = cv2.convexHull(masked_pxs.T)
        cv2.polylines(I, [hull_pxs], True, color, thickness)


if __name__ == '__main__':
    # Then add video generation w/ the matches to the pipeline
    parser = argparse.ArgumentParser(description='project lidar on video')
    parser.add_argument('video', help='video file')
    parser.add_argument('align_data', help='output of align_map.py')
    parser.add_argument('bounds1', help='file with bbox founds of map features')
    parser.add_argument('bounds2', help='file with bbox founds of map features')
    parser.add_argument('outvideo', help='output video file')
    parser.add_argument('--cam', default=2, type=int, help='camera number to use')
    parser.add_argument('--debug', action='store_true', help='debug flag')
    args = parser.parse_args()

    # Read in IMU and matching data

    ad = np.load(args.align_data)
    print ad.files
    start1, start2, all_data1, all_data2 = ad['start1'], ad['start2'], ad['all_data1'], ad['all_data2']
    imu_transforms1, imu_transforms2 = ad['imu_transforms1'], ad['imu_transforms2']
    nn_matches = ad['nn_matches']
    nn_dict = dict()
    for t in range(nn_matches.shape[0]):
        nn_dict[nn_matches[t, 1]] = nn_matches[t, 0]

    # Read in cluster label data

    h5f = h5py.File(args.bounds1, 'r')
    cluster_labels1 = h5f['cluster_labels'][...]
    cluster_centers1 = h5f['cluster_centers'][...]
    h5f.close()
    h5f = h5py.File(args.bounds2, 'r')
    cluster_labels2 = h5f['cluster_labels'][...]
    cluster_centers2 = h5f['cluster_centers'][...]
    h5f.close()

    # Set up video and shared parameters

    cam_num = args.cam
    video_file = args.video
    # PARAM FIXME
    params = LoadParameters('q50_4_3_14_params')
    cam = params['cam'][cam_num-1]
    video_reader = VideoReader(video_file)
    T_from_i_to_l = np.linalg.inv(params['lidar']['T_from_l_to_i'])

    # BGR

    red = [0, 0, 255]
    blue = [255, 0, 0]
    green = [0, 255, 0]

    # Set up video writer

    if not args.debug:
        fps = 30  # PARAM
        fourcc = cv2.cv.CV_FOURCC(*'MJPG')
        video_writer = cv2.VideoWriter()
        video_writer.open(args.outvideo, fourcc, fps, (1280, 960))  # PARAM

    # Read in images from video and project

    t = 0
    while t < min(start1, start2) + MAX_VIDEO_FRAMES:
        # FIXME  What's going on here?
        for count in range(2):
            (success, I) = video_reader.getNextFrame()

        if not success:
            break

        t = video_reader.framenum - 1

        if t not in nn_dict or (t + PROJECT_MAP_WINDOW) not in nn_dict:
            continue

        print t

        # Filter points over a brief time window

        mask_window1 = (all_data1[:, 4] < t + PROJECT_MAP_WINDOW) &\
            (all_data1[:, 4] > t)
        all_data1_copy = np.array(all_data1[mask_window1, :])

        mask_window2 = (all_data2[:, 4] < nn_dict[t + PROJECT_MAP_WINDOW]) &\
            (all_data2[:, 4] > nn_dict[t])
        all_data2_copy = np.array(all_data2[mask_window2, :])

        # Filter centers based on distance since no associated times

        BOUNDS_DIST_TOL = 100  # PARAM
        centers1_mask = np.sqrt(np.sum(
            np.square(cluster_centers1 - imu_transforms1[t, 0:3, 3]), axis=1)) < BOUNDS_DIST_TOL
        centers2_mask = np.sqrt(np.sum(
            np.square(cluster_centers2 - imu_transforms1[t, 0:3, 3]), axis=1)) < BOUNDS_DIST_TOL

        # Reproject and draw points from map

        (pix1, mask1) = localMapToPixels(all_data1_copy, imu_transforms1[t, :, :], T_from_i_to_l, cam)
        (pix2, mask2) = localMapToPixels(all_data2_copy, imu_transforms1[t, :, :], T_from_i_to_l, cam)

        colors1 = np.tile(red, (np.sum(mask1), 1))
        colors2 = np.tile(blue, (np.sum(mask2), 1))
        draw_points(I, pix1, mask1, colors1)
        draw_points(I, pix2, mask2, colors2)

        # Get the cluster labels of points in the time window as well

        cluster_labels1_copy = cluster_labels1.ravel()[mask_window1]
        cluster_labels2_copy = cluster_labels2.ravel()[mask_window2]

        # Reproject and convex hulls

        draw_hulls(I, cluster_labels1_copy, pix1, mask1, red, 2)
        draw_hulls(I, cluster_labels2_copy, pix2, mask2, blue, 2)

        # Draw centers

        (pix1, mask1) = localMapToPixels(cluster_centers1[centers1_mask, :], imu_transforms1[t, :, :], T_from_i_to_l, cam)
        colors1 = np.tile(green, (np.sum(mask1), 1))
        draw_points(I, pix1, mask1, colors1, rad=5)

        (pix2, mask2) = localMapToPixels(cluster_centers2[centers2_mask, :], imu_transforms1[t, :, :], T_from_i_to_l, cam)
        colors2 = np.tile(green, (np.sum(mask2), 1))
        draw_points(I, pix2, mask2, colors2, rad=5)


        # Finally write to video

        if args.debug:
            cv2.imwrite('project_map_on_video_%d.png' % t, I)
            if t > 30:
                break
        else:
            video_writer.write(I)

    print 'Played %d frames' % t
    if not args.debug:
        video_writer.release()
