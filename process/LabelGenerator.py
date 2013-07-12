import preprocess as pp
import preprocess_raw as raw
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
from VideoReader import *
from WGS84toENU import *
import pdb


def formatLabel(orig_label):
    orig_label = imresize(orig_label, (240, 320));
    orig_label = (np.sum(orig_label, axis=2) < 382).astype(float)

    output_label = np.zeros((240, 320))

    start_index = np.argmax(np.max(orig_label, axis=1))
    start_column = np.argmax(orig_label[start_index])
    while start_index < 239 and np.max(orig_label[start_index+1:240]) == 1:
        #print "%d %d" % (start_index, start_column)
        output_label[start_index, start_column] = 1

        end_index = start_index + 1 + np.argmax(np.max(orig_label[start_index+1:240], axis=1))
        end_column = np.argmax(orig_label[end_index])
        for i in range(start_index+1, end_index):
            column = start_column + (i - start_index) * (end_column - start_column) / (end_index - start_index)
            output_label[i, np.round(column)] = 1

        start_index = end_index
        start_column = end_column

    for i in range(start_index, 240):
        output_label[i, start_column] = 1

    return output_label

def runBatch(video_reader, gps_dat, cam, output_base, start_frame, final_frame):
    num_imgs_fwd = 120

    count = 0
    imgs = []
    labels = []
    success = True
    while True:
        (success, I) = video_reader.getNextFrame()
        if count % 160 == 0:
            print count
        if success == False or count == 9600 or start_frame + count == final_frame:
            break

        reshaped = pp.resize(I, (240, 320))[0]
        imgs.append(reshaped)

        framenum = start_frame + count
        label = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam)
        label = formatLabel(label)
        labels.append(label)

        count += 1

    state = random.getstate()
    random.shuffle(imgs)
    random.setstate(state)
    random.shuffle(labels)

    for output_num in range(count / 960):
        merge_imgs = imgs[output_num*960:(output_num+1)*960]
        merge_labels = labels[output_num*960:(output_num+1)*960]

        merge_file = "%s_%d" % (output_base, output_num)
        raw.save_raw_merged_file(merge_file, merge_imgs, merge_labels) 

    print 'finished batch for %s' % output_base
    return (success, count)

def runLabeling(file_path, gps_filename, output_name, frames_to_skip, final_frame):
    video_reader = VideoReader(file_path)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    cam = { }
    cam['R_to_c_from_i'] = np.array([[-1, 0, 0], \
                         [0, 0, -1], \
                         [0, -1, 0]]);
    cam['rot_x'] = deg2rad(-0.66); # -0.569
    cam['rot_y'] = deg2rad(-0.71); # 0.298
    cam['rot_z'] = deg2rad(0.0); # 0.027

    cam['fx'] = 2221.8
    cam['fy'] = 2233.7
    cam['cu'] = 623.7
    cam['cv'] = 445.7
    cam['KK'] = array([[cam['fx'], 0.0, cam['cu']], \
                     [0.0, cam['fy'], cam['cv']], \
                     [0.0, 0.0, 1.0]]);

    more_batches = True
    batch_number = 0

    for i in xrange(frames_to_skip):
        video_reader.getNextFrame()

    start_frame = frames_to_skip
    while more_batches:
        (more_batches, frames) = runBatch(video_reader, gps_dat, cam, "%s_%d" % (output_name, batch_number), start_frame, final_frame)

        batch_number += 1
        print "Done with %d" % batch_number
        start_frame += frames

        if start_frame == final_frame:
            more_batches = False

if __name__ == '__main__':
    folder = sys.argv[1]
    gps_files = glob.glob(folder + '*_gps.out')

    output_name = sys.argv[2]
    frames_to_skip = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    final_frame = int(sys.argv[4]) if len(sys.argv) > 4 else -1
    for gps_file in gps_files:
        prefix = gps_file[0:-8]
        file_path = prefix + '1.avi'
        print file_path
        print gps_file
        runLabeling(file_path, gps_file, output_name + '_' + prefix[-1] + '1', frames_to_skip, final_frame)

