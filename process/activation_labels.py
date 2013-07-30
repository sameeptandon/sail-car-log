import numpy as np
import skimage.feature
import sys
from VideoReader import *
from scipy.ndimage.filters import convolve
from scipy.io import loadmat

sys.path.append('/afs/cs.stanford.edu/u/nikhilb/libs/liblinear-1.93/python')
from liblinearutil import *

import features

def rgb2gray(img):
    (width, height, colors) = img.shape

    return 0.2989 * img[:,:,0] + 0.5870 * img[:,:,1] + 0.1140 * img[:,:,2]

def hogConv(O, weights):
    w, h = O.shape
    im = np.zeros((w, h, 3))
    for i in xrange(3):
        im[:,:,i] = O
    hv = features.hog(im, sbin=16)
    w, h, d = hv.shape
    test = np.reshape(hv, (w*h, d))
    return np.reshape(np.dot(test, weights), (w, h))

def interpolateLanes(x, y):
    xout = -1*np.ones(x.shape)
    yout = -1*np.ones(y.shape)

    for i in [0, 1]:
        lane_x = x[i, :]
        lane_y = y[i, :]

        last_indexed = -1

        for j in xrange(lane_x.size):
            if lane_x[j] != -1:
                xout[i, j] = lane_x[j]
                yout[i, j] = lane_y[j]
                if last_indexed != -1 and last_indexed != j-1:
                    old_x = lane_x[last_indexed]
                    old_y = lane_y[last_indexed]
                    for k in xrange(last_indexed+1, j):
                        xout[i, k] = np.round(old_x + (k - last_indexed) * (lane_x[j] - old_x) / (j - last_indexed))
                        yout[i, k] = np.round(old_y + (k - last_indexed) * (lane_y[j] - old_y) / (j - last_indexed))
                last_indexed = j

        # make sure the end is populated
        endIndex = xout.shape[1] - 1
        if xout[i, endIndex] == -1:
            while endIndex >= 0 and xout[i, endIndex] == -1:
                endIndex -= 1
            x_val = xout[i, endIndex]
            y_val = yout[i, endIndex]

            for j in xrange(endIndex+1, xout.shape[1]):
                xout[i, j] = x_val
                yout[i, j] = y_val

    return (xout, yout)


def findLanes(img, origSize=(960,1280)):
    if len(img.shape) == 3:
        img = rgb2gray(img)
    (rows, cols) = img.shape
    m = np.mean(np.mean(img[rows/2:rows,:]))
    img = abs(img - m);
    max_lane_size = int(np.round(origSize[1] / 36)) # approximation of lane width
    if max_lane_size % 2 == 1:
        max_lane_size += 1
    width_step = 2
    index_steps = int(np.round((float(origSize[0]) / 2) / (max_lane_size / width_step)))

    O = np.zeros((rows, cols))

    lane_width = max_lane_size
    row_index = rows
    while row_index > 0 and lane_width > 0:
        v = np.array([np.concatenate([-1*np.ones(lane_width/2), np.ones(lane_width+1), -1*np.ones(lane_width/2)])])
        v = v/v.size

        start_index = max(0, row_index - index_steps)
        O[start_index:row_index, :] = np.round(convolve(
            img[start_index:row_index, :], v, mode='reflect'))
        lane_width -= width_step
        row_index -= index_steps

    low_vals = O < 0
    O[low_vals] = 0
    high_vals = O > 255
    O[high_vals] = 255

    O_min = np.amin(O)
    O_max = np.amax(O)
    O = (O - O_min) / (O_max - O_min)
    return O


if __name__ == '__main__':
    prev_x = -1*np.ones((2, 1))
    prev_y = -1*np.ones((2, 1))
    consec_borders = np.zeros((2, 1))

    points = np.empty((0,3))

    remove_top = .75
    consec_sides = 5
    min_from_edge = 1
    min_from_side = 3

    skippedPoints = 1
    threshold = .05

    file_path = sys.argv[1]
    video_reader = VideoReader(file_path)

    output_name = sys.argv[2]
    
    count = 0
    from scipy.io import savemat
    while True:
        (success, I) = video_reader.getNextFrame()

        if success == False:
            break
        
        if count % 100 == 0:
            print count
            savemat(output_name, {'points': points})

        O = findLanes(I)

        (rows, cols) = O.shape
        halves = np.array([O[int(np.round(remove_top*rows)):rows, 0:cols/2],
                           O[int(np.round(remove_top*rows)):rows, cols/2:cols]])

        for i in [0, 1]:
            side = halves[i]
            bottom = side[side.shape[0] - 1, :]
            if True:
                edge = np.zeros(side.shape[0])
                if i == 0:
                    edge = side[:, min_from_side]
                else:
                    edge = side[:, cols/2 - min_from_side]
                val = np.amax(edge)
                ind = np.argmax(edge)
                if val > threshold:
                    x_val = 0
                    if i == 0:
                        x_val = min_from_side
                    else:
                        x_val = cols - min_from_side

                    for j in xrange(0, edge.size, skippedPoints):
                        if edge[j] > threshold:
                            val = np.asarray([[x_val, min(j + int(rows*remove_top), rows - min_from_edge), count]])
                            points = np.append(points, val, 0)
            if True:
                y_val = rows - min_from_edge
                for j in xrange(0, bottom.size, skippedPoints):
                    if bottom[j] > threshold:
                        x_val = max(min_from_edge, j) if i == 0 else min(cols/2 + j, cols - min_from_edge)
                        val = np.asarray([[x_val, y_val, count]])
                        points = np.append(points, val, 0)

        count += 1

