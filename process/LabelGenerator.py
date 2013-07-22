import preprocess as pp
import preprocess_many_labels as pml
import glob
import numpy as np
import os
import random
import shutil
import sys
import tempfile
from scipy.misc import toimage, imresize
from GPSReader import *
from GPSReprojection import *
from RandomVideoReader import *
from WGS84toENU import *
import pdb


def formatLabel(orig_label):
    orig_label = (np.sum(orig_label, axis=2) < 765).astype(float) # < 765ish

    output_label = []

    for i in xrange(60):
        positions = []
        for j in xrange(16):
            positions = np.append(positions, np.where(orig_label[16*i + j] == 1))
            

        if len(positions) == 0:
            output_label.append(0)
        else:
            x_pos = int(np.mean(positions) / 16) + 1
            output_label.append(x_pos)

    non_zero = np.where(np.array(output_label) != 0)[0]

    for i in xrange(non_zero.size - 1):
        start = non_zero[i]
        end = non_zero[i+1]
        start_val = output_label[start]
        end_val = output_label[end]
        for j in xrange(start+1, end):
            output_label[j] = start_val + int((j - start) * (end_val - start_val) / (end - start))

    if non_zero.size != 0:
        end = np.max(non_zero)
        end_val = output_label[end]
        for i in xrange(end + 1, len(output_label)):
            output_label[i] = end_val

    return output_label

def runBatch(video_reader, gps_dat, cam, output_base, start_frame, final_frame):
    num_imgs_fwd = 120

    count = 0
    output_num = 0
    imgs = []
    labels = []
    success = True
    while True:
        (success, I, frame) = video_reader.getNextFrame()
        if count % 160 == 0:
            print count
        if success == False:
            break
        if frame < start_frame or (final_frame != -1 and frame >= final_frame):
            continue

        gps_frames = gps_dat[frame:frame+num_imgs_fwd,:]
        if gps_frames.shape[0] < num_imgs_fwd:
            continue

        reshaped = pp.resize(I, (240, 320))[0]
        imgs.append(reshaped)

        framenum = frame
        mask = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam)
        points = formatLabel(mask)
        labels.append(points)

        if len(imgs) == 960:
            merge_file = "%s_%d" % (output_base, output_num)
            pml.save_merged_file(merge_file, imgs, labels)
            imgs = []
            labels = []
            output_num += 1

        count += 1

def runLabeling(file_path, gps_filename, output_name, frames_to_skip, final_frame):
    video_reader = RandomVideoReader(file_path)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    cam = [{}, {}]
    for i in xrange(len(cam)):
        cam[i] = { }
        cam[i]['R_to_c_from_i'] = np.array([[-1, 0, 0], \
                                 [0, 0, -1], \
                                 [0, -1, 0]]);
        cam[i]['fx'] = 2221.8
        cam[i]['fy'] = 2233.7
        cam[i]['cu'] = 623.7
        cam[i]['cv'] = 445.7
        cam[i]['KK'] = array([[cam[i]['fx'], 0.0, cam[i]['cu']], \
                             [0.0, cam[i]['fy'], cam[i]['cv']], \
                             [0.0, 0.0, 1.0]]);
        cam[i]['t_y'] = 1.1
        cam[i]['t_z'] = 0.0


    cam[0]['rot_x'] = deg2rad(-0.8)
    cam[0]['rot_y'] = deg2rad(-0.5)
    cam[0]['rot_z'] = deg2rad(-0.005)
    cam[0]['t_x'] = -0.5

    cam[1]['rot_x'] = deg2rad(-0.62)
    cam[1]['rot_y'] = deg2rad(0.2)
    cam[1]['rot_z'] = deg2rad(0.0)
    cam[1]['t_x'] = 0.5

    cam_to_use = cam[int(output_name[-1]) - 1]

    start_frame = frames_to_skip
    runBatch(video_reader, gps_dat, cam_to_use, output_name, start_frame, final_frame)

    print "Done with %s" % output_name

if __name__ == '__main__':
    folder = sys.argv[1]
    gps_files = glob.glob(folder + '*_gps.out')

    output_name = sys.argv[2]
    frames_to_skip = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    final_frame = int(sys.argv[4]) if len(sys.argv) > 4 else -1
    for gps_file in gps_files:
        prefix = gps_file[0:-8]
        for i in [1,2]:
            file_path = prefix + str(i) + '.avi'
            print file_path
            print gps_file
            runLabeling(file_path, gps_file, output_name + '_' + prefix[-1] + str(i), frames_to_skip, final_frame)

