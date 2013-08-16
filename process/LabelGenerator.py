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

        min_speed = np.min(GPSVelocities(gps_frames))
        if min_speed < 18:  # slower than 40 mph
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
    video_reader.setSubsample(True)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    cam = pickle.load(open('cam_params.pickle', 'rb'))
    cam_to_use = cam[int(output_name[-1]) - 1]

    start_frame = frames_to_skip
    runBatch(video_reader, gps_dat, cam_to_use, output_name, start_frame, final_frame)

    print "Done with %s" % output_name

def parseFolder(args):
    (folder, output_folder) = args
    gps_files = glob.glob(folder +  '*_gps.out')

    frames_to_skip = 0  #int(sys.argv[3]) if len(sys.argv) > 3 else 0
    final_frame = -1  #int(sys.argv[4]) if len(sys.argv) > 4 else -1
    for gps_file in gps_files:
        prefix = gps_file[0:-8]
        for i in [1,2]:
            file_path = prefix + str(i) + '.avi'
            #print file_path
            #print gps_file
            path, output_base = os.path.split(prefix)
            output_name = os.path.join(output_folder, output_base + str(i))
            print output_name
            runLabeling(file_path, gps_file, output_name, frames_to_skip, final_frame)

if __name__ == '__main__':
    folder = (sys.argv[1], sys.argv[2])
    parseFolder(folder)
