import preprocess as pp
import preprocess_many_labels as pml
import glob
import numpy as np
import os
import os.path
import pickle
import random
import shutil
import sys
import tempfile
from scipy.io import loadmat
from scipy.misc import toimage, imresize
from CameraParams import getCameraParams
from CameraReprojection import *
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from RandomVideoReader import *
from WGS84toENU import *
from WarpUtils import *
from WarpedVideoReader import *
import pdb


def outputDistances(distances, framenum, meters_per_point, points_fwd, start_offset):
    output = []
    point_num = 1
    dist = 0

    framenum += 1
    while framenum < distances.size and point_num <= points_fwd:
        dist += distances[framenum]
        if point_num * meters_per_point <= dist - start_offset:
            output.append(framenum)
            point_num += 1
        else:
            framenum += 1
        
    return output

def runBatch(video_reader, gps_dat, cam, output_base, start_frame, final_frame, left_lanes, right_lanes, tr):
    meters_per_point = 10
    points_fwd = 8
    frames_per_second = 50
    distances = GPSVelocities(gps_dat) / frames_per_second
    pts = GPSPos(gps_dat, cam, gps_dat[0, :])[0:2]
    #pts = WGS84toENU(gps_dat[0, 1:4], gps_dat[:, 1:4])[0:2]
    base_vels = np.concatenate((np.array([[0], [0]]), pts[:, 1:] - pts[:, :-1]), axis=1)

    vels = np.array([-1 * base_vels[1, :], base_vels[0, :]])

    count = 0
    output_num = 0
    imgs = []
    labels = []
    success = True
    while True:
        #(success, I, frame, P) = video_reader.getNextFrame()
        (success, I) = video_reader.getNextFrame()
        frame = 10*count
        #frame = count
        P = np.eye(3)
        if count % 160 == 0:
            print count
        if success == False or frame >= tr.shape[0]:
            print success, frame, tr.shape[0]
            break
        if frame < start_frame or (final_frame != -1 and frame >= final_frame):
            continue

        important_frames = (outputDistances(distances, frame, meters_per_point, points_fwd, -5))
        if len(important_frames) < points_fwd:
            continue

        gps_directions = np.transpose(vels[:, important_frames])
        for i in xrange(gps_directions.shape[0]):
            gps_directions[i] = gps_directions[i] / np.linalg.norm(gps_directions[i])

        important_lanes = []
        for fr in important_frames:
            min_val = max(important_frames[0] - 250, 0)
            max_val = min(important_frames[0] + 250, left_lanes.shape[0] - 1)
            distances = np.abs(np.dot(left_lanes[min_val:max_val, 0:2] - pts[:, important_frames[0]], gps_directions[0]))
            important_lanes.append(np.argmin(distances)+min_val)

        important_lanes = np.array(important_lanes)


        max_idx = max(np.max(important_frames), np.max(important_lanes))
        if max_idx >= left_lanes.shape[0] or max_idx >= right_lanes.shape[0]:
            print 'maxing out'
            continue
        temp_left = np.linalg.solve(tr[frame, :, :], left_lanes[important_lanes, :].transpose())
        temp_right = np.linalg.solve(tr[frame, :, :], right_lanes[important_lanes, :].transpose())

        gps_vals = warpPoints(P, GPSColumns(gps_dat[important_frames], cam, gps_dat[frame, :])[0:2])
        left_vals = warpPoints(P, PointsMask(temp_left[0:3, :], cam)[0:2])
        right_vals = warpPoints(P, PointsMask(temp_right[0:3, :], cam)[0:2])
        gps_vals = (gps_vals / 4).astype(np.int32)
        left_vals = (left_vals / 4).astype(np.int32)
        right_vals = (right_vals / 4).astype(np.int32)
        gps_vals[0, gps_vals[0, :] >= 320] = 319
        gps_vals[1, gps_vals[1, :] >= 240] = 239
        left_vals[0, left_vals[0, :] >= 320] = 319
        left_vals[1, left_vals[1, :] >= 240] = 239
        right_vals[0, right_vals[0, :] >= 320] = 319
        right_vals[1, right_vals[1, :] >= 240] = 239
        outputs = []
        
        # scale down column numbers by 16 to aid bucketing but only scale down
        # row numbers by 4 to aid visualization
        for i in xrange(points_fwd):
            outputs.append(left_vals[0, i] / 4)
            outputs.append(gps_vals[0, i] / 4)
            outputs.append(right_vals[0, i] / 4)
            outputs.append(left_vals[1, i])
            outputs.append(gps_vals[1, i])
            outputs.append(right_vals[1, i])
        labels.append(outputs)

        reshaped = pp.resize(I, (240, 320))[0]
        imgs.append(reshaped)


        if len(imgs) == 960:
            merge_file = "%s_%d" % (output_base, output_num)
            pml.save_merged_file(merge_file, imgs, labels, imgRows=(6*points_fwd))
            imgs = []
            labels = []
            output_num += 1

        count += 1

def runLabeling(file_path, gps_filename, output_name, frames_to_skip, final_frame, lp, rp, pickle_loc):
    video_reader = VideoReader(file_path)
    video_reader.setSubsample(True)
    #video_reader.setPerspectives(pickle_loc)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    cam = getCameraParams()
    cam_to_use = cam[int(output_name[-1]) - 1]

    lp = pixelTo3d(lp, cam_to_use)
    rp = pixelTo3d(rp, cam_to_use)
    tr = GPSTransforms(gps_dat, cam_to_use)
    pitch = -cam_to_use['rot_x']
    height = 1.106
    R_camera_pitch = euler_matrix(cam_to_use['rot_x'], cam_to_use['rot_y'], cam_to_use['rot_z'], 'sxyz')[0:3, 0:3]
    Tc = np.eye(4)
    Tc[0:3, 0:3] = R_camera_pitch.transpose()
    Tc[0:3, 3] = [-0.2, -height, -0.5]
    lpts = np.zeros((lp.shape[0], 4))
    rpts = np.zeros((rp.shape[0], 4))
    for t in range(min(tr.shape[0], lp.shape[0])):
        lpts[t, :] = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([lp[t, 0], lp[t, 1], lp[t, 2], 1])))
        rpts[t, :] = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([rp[t, 0], rp[t, 1], rp[t, 2], 1])))

    start_frame = frames_to_skip
    runBatch(video_reader, gps_dat, cam_to_use, output_name, start_frame, final_frame, lpts, rpts, tr)

    print "Done with %s" % output_name

def parseFolder(args):
    (folder, output_folder, output_prefix, laneLoc, pickleLoc) = args
    gps_files = glob.glob(folder +  '*_gps.out')

    frames_to_skip = 0  #int(sys.argv[3]) if len(sys.argv) > 3 else 0
    final_frame = -1  #int(sys.argv[4]) if len(sys.argv) > 4 else -1
    for gps_file in gps_files:
        prefix = gps_file[0:-8]
        for i in [1,2]:
            file_path = prefix + str(i) + '.avi'
            
            path, output_base = os.path.split(prefix)
            pts_file = laneLoc + output_base + str(i) + '-interpolated.mat'
            if not os.path.isfile(pts_file):
                return
            pts = loadmat(pts_file)
            output_base = output_prefix + output_base
            output_name = os.path.join(os.path.join(output_folder, str(i)), output_base + str(i))
            print output_name
            lp = pts['left']
            rp = pts['right']

            runLabeling(file_path, gps_file, output_name, frames_to_skip, final_frame, lp, rp, pickleLoc)

if __name__ == '__main__':
    folder = (sys.argv[1], sys.argv[2], '', sys.argv[3], sys.argv[4])
    parseFolder(folder)
