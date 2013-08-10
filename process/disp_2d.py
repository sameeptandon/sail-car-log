import numpy as np
import sys
from cv2 import imread, imshow, resize, waitKey
from scipy.misc import imresize
from scipy.io import loadmat
from GPSReader import *
from GPSTransforms import *
from generate_lane_labels import *
from VideoReader import *

if __name__ == '__main__':
    video_filename = sys.argv[1]
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    num_imgs_fwd = 125
    video_reader = VideoReader(video_filename)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    labels = loadmat(sys.argv[2])
    lp = labels['left']
    rp = labels['right']

    

    cam = { }
    cam['R_to_c_from_i'] = array([[-1, 0, 0], \
                         [0, 0, -1], \
                         [0, -1, 0]]);

    if cam_num == 1:
        cam['rot_x'] = deg2rad(-0.8); # better cam 1
        cam['rot_y'] = deg2rad(-0.5);
        cam['rot_z'] = deg2rad(-0.005);
        cam['t_x'] = -0.5;
        cam['t_y'] = 1.1;
        cam['t_z'] = 0.0;
    elif cam_num == 2:
        cam['rot_x'] = deg2rad(-0.61); # better cam 2 
        cam['rot_y'] = deg2rad(0.2);
        cam['rot_z'] = deg2rad(0.0);
        cam['t_x'] = 0.5;
        cam['t_y'] = 1.1;
        cam['t_z'] = 0.0;

    cam['fx'] = 2221.8
    cam['fy'] = 2233.7
    cam['cu'] = 623.7
    cam['cv'] = 445.7
    cam['KK'] = array([[cam['fx'], 0.0, cam['cu']], \
                     [0.0, cam['fy'], cam['cv']], \
                     [0.0, 0.0, 1.0]]);

    tr = GPSTransforms(gps_dat, cam)

    # probably have to change these
    pitch = 0.0
    height = 1.106
    p2 = 0.00
    f = (cam['fx'] + cam['fy']) / 2
    R_to_c_from_i = cam['R_to_c_from_i']
    R_camera_pitch = euler_matrix(cam['rot_x'], cam['rot_y'],\
            cam['rot_z'], 'sxyz')[0:3,0:3]
    R_to_c_from_i = dot(R_camera_pitch, R_to_c_from_i)
    Tc = np.eye(4)
    Tc[0:3, 0:3] = np.transpose(R_to_c_from_i)
    #Tc[1, 3] -= height
    Tc = np.array([[1, 0, 0, 0], [0, np.cos(pitch), -np.sin(pitch), -height], [0, np.sin(pitch), np.cos(pitch), 0], [0, 0, 0, 1]])

    Tc2 = np.eye(4) # check testTrackReverse for actual transformation value

    count = 0
    start = 0
    while True:
        (success, I) = video_reader.getNextFrame()

        if not success:
            break
        if count % 5 != 0: 
            count += 1
            continue

        if count > lp.shape[0] or count > rp.shape[0]:
            break

        start = count
        left_points = np.zeros((960, 1280))
        right_points = np.zeros((960, 1280))
        for points_count in xrange(start, min(lp.shape[0], start+num_imgs_fwd)):
            pt = lp[points_count]
            Z = ((pt[1]-cam['cv'])*sin(pitch)*height+f*cos(pitch)*height)/(cos(pitch)*(pt[1]-cam['cv'])-f*sin(pitch))
            X = (cos(pitch)*Z-sin(pitch)*height)*(pt[0]-cam['cu'])/f
            Y = 0
            Pos = np.dot(tr[points_count, :, :], np.linalg.solve(Tc, np.array([X, Y, Z, 1])))
            Pos2 = np.linalg.solve(tr[count, :, :], Pos)

            if pt[0] != -1:
                pos2 = np.round(np.dot(cam['KK'], Pos2[0:3]) / Pos2[2])
                if not (pos2[1] < 0 or pos2[0] < 0 or pos2[1] >= 960 or pos2[0] >= 1280):
                    left_points[pos2[1], pos2[0]] += 1

                if pos2[1] > 3 and pos2[1] < 957 and pos2[0] > 3 and pos2[0] < 1277:
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 0] = 0
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 1] = 0
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 2] = 255

            pt = rp[points_count]
            Z = ((pt[1]-cam['cv'])*sin(pitch)*height+f*cos(pitch)*height)/(cos(pitch)*(pt[1]-cam['cv'])-f*sin(pitch))
            X = (cos(pitch)*Z-sin(pitch)*height)*(pt[0]-cam['cu'])/f
            Y = 0
            Pos = np.dot(tr[points_count, :, :], np.linalg.solve(Tc, np.array([X, Y, Z, 1])))
            Pos2 = np.linalg.solve(tr[count, :, :], Pos)

            if pt[0] != -1:
                pos2 = np.round(np.dot(cam['KK'], Pos2[0:3]) / Pos2[2])
                if not (pos2[1] < 0 or pos2[0] < 0 or pos2[1] >= 960 or pos2[0] >= 1280):
                    right_points[pos2[1], pos2[0]] += 1

                if pos2[1] > 3 and pos2[1] < 957 and pos2[0] > 3 and pos2[0] < 1277:
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 0] = 0
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 1] = 0
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 2] = 255

        step_size = 1
        polynomial_fit = 3
        sum_thresh = 1
        thickness = 2


        avg_left = np.dot(left_points, range(0, 1280))
        left_count = np.sum(left_points, axis=1)
        left_count[left_count == 0] = 1
        avg_left = avg_left / left_count

        left_control = np.zeros((0, 2))
        for y in xrange(0, 960, step_size):
            left_window = np.zeros((6, step_size))
            for i in xrange(0, step_size):
                index = y+i
                if index >= 960:
                    break
                pos = avg_left[index]
                if pos >= 3 and pos <= 1277:
                    left_window[:, i] = left_points[index, pos-3:pos+3]
            max_sum = np.max(np.sum(left_window, axis=0))
            if max_sum >= sum_thresh:
                y_index = y+np.argmax(np.sum(left_window, axis=0))
                x_index = avg_left[y_index]
                left_control = np.append(left_control, np.array([[x_index, y_index]]), axis=0)

        left_x = left_control[:, 0]
        left_y = left_control[:, 1]
        if left_x.size > 3:
            p = np.polyfit(left_y, left_x, polynomial_fit)
            #left_spline = UnivariateSpline(left_y, left_x, s=smoothness, k=3, bbox=[left_y[0], 960])
            y_output = range(int(np.min(left_y)), int(np.max(left_y) + 1))
            x_output = np.interp(y_output, left_y, left_x);
            #x_output = left_spline(y_output)
            for y in range(len(y_output)):
                y_val = y_output[y]
                #x_val = p[polynomial_fit]
                x_val = x_output[y]
                """
                for x in xrange(polynomial_fit):
                    x_val += p[x] * y_val**(polynomial_fit-x)
                if x_val < 0 or x_val >= 1280:
                    continue
                """
                pos2 = [x_val, y_val]
                #thickness = 5 + max(0, (y_val-480)*18/480)
                #I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 0] = 255
                #I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 1] = 0
                #I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 2] = 0
            # slope = p[polynomial_fit-1]
            # y_max = int(np.max(left_y))
            # x_max = p[polynomial_fit]
            # for x in xrange(polynomial_fit):
            #     x_max += p[x] * y_max**(polynomial_fit-x)
            # for x in xrange(polynomial_fit-1):
            #     slope += (polynomial_fit - x) * p[x] * y_max**(polynomial_fit-1-x)
            # for y in range(y_max, 960):
            #     x_val = (y - y_max) * slope + x_max
            #     if x_val < 0 or x_val >= 1280:
            #         continue
            #     pos2 = [x_val, y]
            #     I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 0] = 255
            #     I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 1] = 0
            #     I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 2] = 0

        avg_right = np.dot(right_points, range(0, 1280))
        right_count = np.sum(right_points, axis=1)
        right_count[right_count == 0] = 1
        avg_right = avg_right / right_count

        right_control = np.zeros((0, 2))
        for y in xrange(0, 960, step_size):
            right_window = np.zeros((6, step_size))
            for i in xrange(0, step_size):
                index = y+i
                if index >= 960:
                    break
                pos = avg_right[index]
                if pos >= 3 and pos <= 1277:
                    right_window[:, i] = right_points[index, pos-3:pos+3]
            max_sum = np.max(np.sum(right_window, axis=0))
            if max_sum >= sum_thresh:
                y_index = y+np.argmax(np.sum(right_window, axis=0))
                x_index = avg_right[y_index]
                right_control = np.append(right_control, np.array([[x_index, y_index]]), axis=0)

        right_x = right_control[:, 0]
        right_y = right_control[:, 1]
        if right_x.size > 3:
            p = np.polyfit(right_y, right_x, polynomial_fit)
            #right_spline = UnivariateSpline(right_y, right_x, s=smoothness, k=3, bbox=[right_y[0], 960])
            y_output = range(int(np.min(right_y)), int(np.max(right_y) + 1))
            #x_output = right_spline(y_output)
            for y in range(len(y_output)):
                y_val = y_output[y]
                x_val = p[polynomial_fit]
                for x in xrange(polynomial_fit):
                    x_val += p[x] * y_val**(polynomial_fit-x)
                if x_val < 0 or x_val >= 1280:
                    continue
                pos2 = [x_val, y_val]
                #thickness = 5 + max(0, (y_val-480)*18/480)
                #I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 0] = 0
                #I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 1] = 255
                #I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 2] = 0
            # slope = p[polynomial_fit-1]
            # y_max = int(np.max(right_y))
            # x_max = p[polynomial_fit]
            # for x in xrange(polynomial_fit):
            #     x_max += p[x] * y_max**(polynomial_fit-x)
            # for x in xrange(polynomial_fit-1):
            #     slope += (polynomial_fit - x) * p[x] * y_max**(polynomial_fit-1-x)
            # for y in range(y_max, 960):
            #     x_val = (y - y_max) * slope + x_max
            #     if x_val < 0 or x_val >= 1280:
            #         continue
            #     pos2 = [x_val, y]
            #     I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 0] = 0
            #     I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 1] = 255
            #     I[pos2[1]-thickness:pos2[1]+thickness, pos2[0]-thickness:pos2[0]+thickness, 2] = 0

        count += 1
        I = imresize(I, (720, 960))
        imshow('video', I)
        key = waitKey(10)
        if key == ord('q'):
            break
