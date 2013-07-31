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
    img = img - m
    img[img < 0] = 0
    max_lane_size = int(np.round(origSize[1] / 36)) # approximation of lane width
    if max_lane_size % 2 == 1:
        max_lane_size += 1
    width_step = 2
    index_steps = int(np.round((float(origSize[0]) / 2) / (max_lane_size / width_step)))

    O = np.zeros((rows, cols))

    lane_width = max_lane_size
    row_index = rows
    while row_index > 0 and lane_width > 2:
        v = 4*np.array([np.concatenate([-1*np.ones(lane_width/2), np.ones(lane_width+1), -1*np.ones(lane_width/2)])])
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

    x_all = np.empty((2,0))
    y_all = np.empty((2,0))

    x_gappy = np.empty((2,0))
    y_gappy = np.empty((2,0))

    remove_top = .75
    consec_sides = 5
    min_from_edge = 1
    min_from_side = 3

    threshold = .25

    file_path = sys.argv[1]
    video_reader = VideoReader(file_path)

    model_loc = sys.argv[2]
    m = load_model(model_loc)
    weights = []
    for i in xrange(m.get_nr_feature()):
        weights.append(m.w[i])

    weights = np.asarray(weights)
    weights = np.reshape(weights, (weights.size, 1))

    output_name = sys.argv[3]
    
    count = 0
    while True:
        (success, I) = video_reader.getNextFrame()
        if success == False:
            break
        
        if count % 100 == 0:
            print count

        if count == 1000:
            break

        """
        if count == 24:
            import scipy.misc
            scipy.misc.imsave('question.png', rgb2gray(I));
            break
        """

        O = findLanes(I)
        O = hogConv(O, weights)
        """
        h,v = hogConv(O, w)
        from matplotlib import pyplot as plt
        import matplotlib.cm as cm
        plt.imshow(I)
        plt.pause(5)
        plt.imshow(O, cmap = cm.Greys_r)
        plt.pause(5)
        plt.imshow(h, cmap = cm.Greys_r)
        plt.pause(5)
        break
        """
        (rows, cols) = O.shape
        halves = np.array([O[int(np.round(remove_top*rows)):rows, 0:cols/2],
                           O[int(np.round(remove_top*rows)):rows, cols/2:cols]])

        next_x_gappy = -1*np.ones((2,1))
        next_y_gappy = -1*np.ones((2,1))
        next_x_all = -1*np.ones((2,1))
        next_y_all = -1*np.ones((2,1))
        for i in [0, 1]:
            set_previous = False
            side = halves[i]
            bottom = side[side.shape[0] - 1, :]
            val = np.amax(bottom)
            ind = np.argmax(bottom)
            if val <= threshold:
                edge = np.zeros(side.shape[0])
                if i == 0:
                    edge = side[:, min_from_side]
                else:
                    edge = side[:, cols/2 - min_from_side]
                val = np.amax(edge)
                ind = np.argmax(edge)
                if val > threshold:
                    consec_borders[i] += 1
                    if consec_borders[i] >= consec_sides:
                        if prev_y[i] == -1:
                            set_previous = True
                        prev_y[i] = min(ind + int(rows*remove_top), rows - min_from_edge)
                        if i == 0:
                            prev_x[i] = min_from_side
                        else:
                            prev_x[i] = cols - min_from_side

                        next_x_gappy[i] = prev_x[i]
                        next_y_gappy[i] = prev_y[i]
            else:
                consec_borders[i] = 0
                if prev_y[i] == -1:
                    set_previous = True
                prev_y[i] = rows - min_from_edge
                if i == 0:
                    prev_x[i] = max(min_from_edge, ind)
                else:
                    prev_x[i] = min(cols/2 + ind, cols - min_from_edge)

                next_x_gappy[i] = prev_x[i]
                next_y_gappy[i] = prev_y[i]

            if set_previous:
                for j in xrange(count):
                    x_gappy[i,j] = prev_x[i]
                    y_gappy[i,j] = prev_y[i]
                    x_all[i,j] = prev_x[i]
                    y_all[i,j] = prev_y[i]

            next_x_all[i] = prev_x[i]
            next_y_all[i] = prev_y[i]

        x_gappy = np.append(x_gappy, next_x_gappy, 1)
        y_gappy = np.append(y_gappy, next_y_gappy, 1)

        x_all = np.append(x_all, next_x_all, 1)
        y_all = np.append(y_all, next_y_all, 1)
        
        count += 1

    print x_gappy.shape
    print y_gappy.shape
    print x_all.shape
    print y_all.shape

    (x_filled, y_filled) = interpolateLanes(x_gappy, y_gappy)

    from scipy.io import savemat
    savemat(output_name, {'xall': x_filled, 'yall': y_filled})
