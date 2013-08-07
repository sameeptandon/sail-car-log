import cv
import cv2
import numpy as np
import sys
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from WGS84toENU import *
from generate_lane_labels import *
from scipy.io import savemat


if __name__ == '__main__':
    prev_x = -1*np.ones((2, 1))
    prev_y = -1*np.ones((2, 1))
    consec_borders = np.zeros((2, 1))

    right_points = np.zeros((0, 2))
    left_points = np.zeros((0, 2))

    remove_top = .75
    consec_sides = 5
    min_from_edge = 1
    min_from_side = 3

    skippedPoints = 1
    threshold = .05

    video_filename = sys.argv[1]
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    num_imgs_fwd = 25
    video_reader = VideoReader(video_filename)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    output_name = sys.argv[2]

    pitch = 0
    height = 1.106
    p2 = 0.00

    cam = {}
    cam['R_to_c_from_i'] = np.array([[-1, 0, 0],
                                     [0, 0, -1],
                                     [0, -1, 0]])

    if cam_num == 1:
        cam['rot_x'] = deg2rad(-0.8)  # better cam 1
        cam['rot_y'] = deg2rad(-0.5)
        cam['rot_z'] = deg2rad(-0.005)
        cam['t_x'] = -0.5
        cam['t_y'] = 1.1
        cam['t_z'] = 0.0
    elif cam_num == 2:
        cam['rot_x'] = deg2rad(-0.61)  # better cam 2
        cam['rot_y'] = deg2rad(0.2)
        cam['rot_z'] = deg2rad(0.0)
        cam['t_x'] = 0.5
        cam['t_y'] = 1.1
        cam['t_z'] = 0.0

    cam['fx'] = 2221.8
    cam['fy'] = 2233.7
    cam['cu'] = 623.7
    cam['cv'] = 445.7
    cam['KK'] = array([[cam['fx'], 0.0, cam['cu']],
                       [0.0, cam['fy'], cam['cv']],
                       [0.0, 0.0, 1.0]])

    Tc = np.array([[1, 0, 0, 0], [0, np.cos(pitch), -np.sin(pitch), -height], [0, np.sin(pitch), np.cos(pitch), 0], [0, 0, 0, 1]])
    f = (cam['fx'] + cam['fy']) / 2

    tr = GPSTransforms(gps_dat, cam)
    src = np.array([[499, 597], [972, 597], [1112, 661], [448, 678]], np.float32) / 4
    dst = np.array([[320, 320], [960, 320], [960, 640], [320, 640]], np.float32) / 4
    P = cv2.getPerspectiveTransform(src, dst)

    lastCols = [None, None]

    edge_size = 100
    count = 0
    ratio = 4
    while True:
        (success, I) = video_reader.getNextFrame()

        if success is False:
            break

        if count % 100 == 0:
            print count
            savemat(output_name, {'left': left_points, 'right': right_points})

        #if count == 5000:
        #    break

        imsize = (320, 240)
        I = cv2.resize(I, imsize)
        if lastCols[0] is None:
            M = GPSMask(gps_dat[count:count+num_imgs_fwd], cam, width=1)
            M = 255 - cv2.resize(M, imsize)
            warped_M = np.nonzero(cv2.warpPerspective(M, P, imsize))
            col_avg = np.mean(warped_M[1])
            lastCols[0] = col_avg - 50
            lastCols[1] = col_avg + 50
        
        (O, lastCols) = findLanes(I, (imsize[1], imsize[0]), lastCols, P)

        """
        Instead of the below, check the borders for an activation and if
        one is found, store only that point. (as x, y, z, 1, framenum, 1)
        If not, store it as (-1, -1, -1, 0, framenum, 0)

        Later, you'll interpolate the (x, y, z) for each discovered point
        and from there you can treat it however
        """
        mask = O[:, :, 2] > 0
        I[mask, 0] = 0
        I[mask, 1] = 0
        I[mask, 2] = 255

        bottom_vals = np.zeros((240, 320, 3))
        bottom_vals[238:239, :, 2] = 255
        bottom_vals = cv2.warpPerspective(bottom_vals, P, imsize, flags=cv.CV_WARP_INVERSE_MAP)

        O_bin = np.copy(O[:, :, 2])

        O_bin[bottom_vals[:, :, 2] == 0] = 0
        O_bleft = O_bin[:, 0:160]
        O_bright = O_bin[:, 160:320]

        bottom_left = np.unravel_index(np.argmax(O_bleft), (240, 160))
        bottom_left = (bottom_left[0] * ratio + ratio, bottom_left[1] * ratio + ratio)
        bottom_right = np.unravel_index(np.argmax(O_bright), (240, 160))
        bottom_right = (bottom_right[0] * ratio + ratio, (bottom_right[1] + 160) * ratio + ratio)

        left_activations = np.argmax(O[:, 2, 2])
        right_activations = np.argmax(O[:, 317, 2])

        new_points = np.zeros((0, 3))
        right_point = np.array([-1, -1])
        left_point = np.array([-1, -1])
        if np.max(O_bleft) > 0:
            left_point[0] = bottom_left[1]
            left_point[1] = bottom_left[0]
        if np.max(O_bright) > 0:
            right_point[0] = bottom_right[1]
            right_point[1] = bottom_right[0]

        if left_point[0] == -1 and np.max(O[:, 2, 2]) > 0:
            left_point[0] = 4
            left_point[1] = left_activations * ratio + ratio

        if right_point[0] == -1 and np.max(O[:, 317, 2]) > 0:
            right_point[0] = 1276
            right_point[1] = right_activations * ratio + ratio

        left_points = np.append(left_points, left_point.reshape((1, left_point.size)), axis=0)
        right_points = np.append(right_points, right_point.reshape((1, right_point.size)), axis=0)

        """
        if right_point[5] == 1:
            Pos2 = np.linalg.solve(tr[count, :, :], right_point[0:4])
            pos2 = np.round(np.dot(cam['KK'], Pos2[0:3]) / Pos2[2] / ratio)
            pos2[1] = max(pos2[1], 4)
            pos2[1] = min(pos2[1], 237)
            pos2[0] = min(pos2[0], 317)
            pos2[0] = max(pos2[0], 4)
            print 'right', pos2, right_point
            if pos2[1] > 3 and pos2[1] < 237 and pos2[0] > 3 and pos2[0] < 317:
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 0] = 255
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 1] = 0
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 2] = 0
        if left_point[5] == 1:
            Pos2 = np.linalg.solve(tr[count, :, :], left_point[0:4])
            pos2 = np.round(np.dot(cam['KK'], Pos2[0:3]) / Pos2[2])
            pos2 = pos2 / ratio
            pos2[1] = max(pos2[1], 4)
            pos2[1] = min(pos2[1], 237)
            pos2[0] = min(pos2[0], 317)
            pos2[0] = max(pos2[0], 4)
            print 'left', pos2, left_point
            if pos2[1] > 3 and pos2[1] < 237 and pos2[0] > 3 and pos2[0] < 317:
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 0] = 0
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 1] = 255
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 2] = 0

        cv2.imshow('video', I)
        key = cv2.waitKey()
        if key == ord('q'):
            break
        """
        count += 1
