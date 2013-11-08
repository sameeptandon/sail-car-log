import glob
import preprocess as pp
import preprocess_velocities as pml
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
from cv2 import imshow, waitKey, resize
import pdb
from copy import deepcopy

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

def runBatch(video_reader, gps_dat, cam, output_base, start_frame, final_frame, left_lanes, right_lanes, ldist, rdist, tr):
    meters_per_point = 10
    points_fwd = 8
    frames_per_second = 50
    distances = GPSVelocities(gps_dat) / frames_per_second
    pts = GPSPos(gps_dat, cam, gps_dat[0, :]) # points wrt camera
    count = 0
    output_num = 0
    imgs = []
    labels = []
    success = True
    while True:
        (success, I, frame, P) = video_reader.getNextFrame()
        #P = np.eye(3)
        if count % 160 == 0:
            print count
        if success == False or frame >= tr.shape[0]:
            print success, frame, tr.shape[0]
            break
        if frame < start_frame or (final_frame != -1 and frame >= final_frame):
            continue

        important_frames = (outputDistances(distances, frame, meters_per_point, points_fwd, 2))
        if len(important_frames) < points_fwd:
            continue

        important_left = []
        important_right = []

        velocities = gps_dat[important_frames,4:7]
        velocities[:,[0, 1]] = velocities[:,[1, 0]]
        sideways = np.cross(velocities, np.array([0,0,1]), axisa=1)
        sideways/= np.sqrt((sideways ** 2).sum(-1))[..., np.newaxis]
        vel_start = ENU2IMU(np.transpose(velocities), gps_dat[0,:])
        vel_current = ENU2IMU(np.transpose(velocities), gps_dat[frame,:])
        sideways_start = GPSPosCamera(np.cross(vel_start, np.array([0,0,1]), axisa=0).transpose(), cam) # sideways vector wrt starting frame (camera)
        sideways_current = GPSPosCamera(np.cross(vel_current, np.array([0,0,1]), axisa=0).transpose(), cam) # sideways vector wrt starting frame (camera)
        sideways_start /= np.sqrt((sideways_start ** 2).sum(0))[np.newaxis, ...]
        sideways_current /= np.sqrt((sideways_current ** 2).sum(0))[np.newaxis, ...] # save sideways_current
        center = GPSPos(gps_dat[important_frames,:], cam, gps_dat[0, :]) # points wrt imu
        for ind in xrange(len(important_frames)):
            fr = important_frames[ind]
            min_val = max(fr - 50, 0)
            max_val = min(fr + 50, left_lanes.shape[0] - 1)

            l_distances = np.cross(left_lanes[min_val:max_val, 0:3] - np.transpose(center[:,ind]), np.transpose(sideways_start[:, ind]), axisa=1)
            l_distances = np.sqrt((l_distances ** 2).sum(-1))
            r_distances = np.cross(right_lanes[min_val:max_val, 0:3] - np.transpose(center[:,ind]), np.transpose(sideways_start[:, ind]), axisa=1)
            r_distances = np.sqrt((r_distances ** 2).sum(-1))
            important_left.append(np.argmin(l_distances)+min_val)
            important_right.append(np.argmin(r_distances)+min_val)
        important_left = np.array(important_left)
        important_right = np.array(important_right)
        max_idx = max(max(np.max(important_frames), np.max(important_left)), np.max(important_right))
        if max_idx >= left_lanes.shape[0] or max_idx >= right_lanes.shape[0]:
            print 'maxing out'
            continue

        temp_left = np.linalg.solve(tr[frame, :, :], left_lanes[important_left, :].transpose()) # save temp_left [0:3,:]
        #temp_left[0:3, :] += sideways_current*0.8

        temp_gps = GPSPos(gps_dat[important_frames], cam, gps_dat[frame, :]) # save temp_gps [0:3,:]

        temp_right = np.linalg.solve(tr[frame, :, :], right_lanes[important_right, :].transpose()) # save temp_right [0:3,:]
        #temp_right[0:3, :] -= sideways_current*0.8

        outputs = []
        for i in xrange(temp_left.shape[1]):
            outputs.append(temp_left[0, i])
            outputs.append(temp_left[1, i])
            outputs.append(temp_left[2, i])
        for i in xrange(temp_gps.shape[1]):
            outputs.append(temp_gps[0, i])
            outputs.append(temp_gps[1, i])
            outputs.append(temp_gps[2, i])
        for i in xrange(temp_left.shape[1]):
            outputs.append(temp_right[0, i])
            outputs.append(temp_right[1, i])
            outputs.append(temp_right[2, i])

        for i in xrange(sideways_current.shape[1]):
            outputs.append(sideways_current[0, i])
            outputs.append(sideways_current[1, i])
            outputs.append(sideways_current[2, i])

        for i in xrange(P.shape[0]):
            for j in xrange(P.shape[1]):
                outputs.append(P[i, j])

        labels.append(outputs)
        
        
        reshaped = pp.resize(I, (240, 320))[0]
        imgs.append(reshaped)
        if len(imgs) == 960:
            merge_file = "%s_%d" % (output_base, output_num)
            pml.save_merged_file(merge_file, imgs, labels, imgRows=len(labels[0]))
            imgs = []
            labels = []
            output_num += 1

        count += 1

def runLabeling(file_path, gps_filename, output_name, frames_to_skip, final_frame, lp, rp, pickle_loc):
    video_reader = WarpedVideoReader(file_path)
    #video_reader.setSubsample(True)
    video_reader.setPerspectives(pickle_loc)
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

    ldist = np.apply_along_axis(np.linalg.norm, 1, np.concatenate((np.array([[0, 0, 0, 0]]), lpts[1:] - lpts[0:-1])))
    rdist = np.apply_along_axis(np.linalg.norm, 1, np.concatenate((np.array([[0, 0, 0, 0]]), rpts[1:] - rpts[0:-1])))
    start_frame = frames_to_skip
    runBatch(video_reader, gps_dat, cam_to_use, output_name, start_frame, final_frame, lpts, rpts, ldist, rdist, tr)

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
