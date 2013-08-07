import numpy as np
import skimage.feature
import sys
from VideoReader import *
from scipy.ndimage.filters import convolve
from scipy.io import loadmat
import cv
import cv2

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


def findLanes(img, origSize=(960,1280), lastCols=[None, None], lastLine=[None,None,None,None], P=np.eye(3)):
    #if len(img.shape) == 3:
    #    img = rgb2gray(img)
    (rows, cols, channels) = img.shape
    
    img = cv2.warpPerspective(img, P, (cols, rows))

    # mean subtraction on image, clamp to 0
    m = np.mean(np.mean(img[:,:],axis=0),axis=0)
    img = img - m
    img[img < 0] = 0

    # set max_lane_size to about 20 in the 1280x960 image
    max_lane_size = int(np.round(origSize[1] / 64)) # approximation of lane width
    if max_lane_size % 2 == 1:
        max_lane_size += 1
    width_step = 2
    index_steps = int(np.round((float(origSize[0]) / 2) / (max_lane_size / width_step)))

    O = np.zeros((rows, cols, channels))
    lane_width = max_lane_size
    row_index = rows
    v = 4*np.array([np.concatenate([-1*np.ones(lane_width/2), 1*np.ones(lane_width+1), -1*np.ones(lane_width/2)])])
    v = v/v.size
    start_index = max(0, row_index - index_steps)
    O_1 = np.round(convolve(img[:,:,0], v, mode='reflect')).reshape((rows,cols,1)) 
    O_2 = np.round(convolve(img[:,:,1], v, mode='reflect')).reshape((rows,cols,1)) 
    O_3 = np.round(convolve(img[:,:,2], v, mode='reflect')).reshape((rows,cols,1)) 

    O = cv2.merge([O_1, O_2, O_3])

   
    """
    #idea for windowing around current lastCol, but could be bad as possible
    to not recover on a bad misdetection
    if lastCols[0] != None and lastCols[1] != None:
        right_lane_min_x = lastCols[1]-40;
        right_lane_max_x = lastCols[1]+40;
        left_lane_min_x = lastCols[0]-40;
        left_lane_max_x = lastCols[0]+40;
        O[:,0:left_lane_min_x] = 0
        O[:,left_lane_max_x:right_lane_min_x] = 0
        O[:,right_lane_max_x:] = 0
    """

    """
    #mean subtract output image
    m = np.mean(np.mean(O[rows/2:rows,:],axis=0),axis=0)
    O = O - m
    O[O < 0] = 0
    """

    # thresholding for lane detection
    white_lane_detect = np.sum(O,axis=2) > 350
    yellow_lane_detect = np.logical_and(O[:,:,1] + O[:,:,2] > 120, O[:,:,0] < 50) 
    low_vals = np.logical_and(np.logical_not(white_lane_detect), np.logical_not(yellow_lane_detect))
    O[low_vals,:] = -0

    # get rid of top of the image so we don't find a column fit to it
    O[0:2*rows/4,:] = 0
    #return(O,lastCols)

    # go to gray scale for convenience
    O_gray = rgb2gray(O);

    # if you have lastCols, find the midpoint and submidpoints for 
    # zero-ing the center of the lane
    if lastCols[0] is not None and lastCols[1] is not None:
        midpoint_lastCols = 0.5*(lastCols[0] + lastCols[1])
        mid_left = 0.5*(lastCols[0] + midpoint_lastCols);
        mid_right = 0.5*(lastCols[1] + midpoint_lastCols);
        O_gray[:, mid_left:mid_right] = 0
    else:
        midpoint_lastCols = cols/2

    # compute the sum of activations in each column and find the max 
    # responding column on the left and right sides
    top_k = 50
    column_O = np.sum(O_gray,axis=0);
    top_k = min(top_k, np.nonzero(column_O[0:midpoint_lastCols])[0].size)
    top_k = min(top_k, np.nonzero(column_O[midpoint_lastCols:])[0].size)

    resp_left = np.copy(column_O[0:midpoint_lastCols].argsort()[-top_k:][::-1])
    min_left = np.argmax(np.abs(resp_left)) # closest to midpoint
    resp_left = resp_left[min_left]

    resp_right = np.copy(column_O[midpoint_lastCols:].argsort()[-top_k:][::-1])
    min_right = np.argmin(np.abs(resp_right)) #closest to midpoint
    resp_right = resp_right[min_right] + midpoint_lastCols

    #resp_left = np.argmax(column_O[0: midpoint_lastCols]);
    #resp_right = np.argmax(column_O[midpoint_lastCols:]) + midpoint_lastCols;


    # was there a detection on either side? 
    LEFT_LANE_DETECTION = np.max(column_O[0: midpoint_lastCols]) != 0
    RIGHT_LANE_DETECTION = np.max(column_O[midpoint_lastCols:]) != 0

    if not LEFT_LANE_DETECTION:
        resp_right = lastCols[1]
    if not RIGHT_LANE_DETECTION:
        resp_left = lastCols[0]

    # how to move the columns based on previous cols 
    if LEFT_LANE_DETECTION:
        if abs(lastCols[0] - resp_left) < 2:
            lastCols[0] = 0.2*resp_left + 0.8*lastCols[0];
        elif abs(lastCols[0] - resp_left) < 5:
            lastCols[0] = 0.15*resp_left + 0.85*lastCols[0];
        elif abs(lastCols[0] - resp_left) < 10:
            lastCols[0] = 0.15*resp_left + 0.85*lastCols[0];
        #elif abs(lastCols[0] - resp_left) < 20:
        else:
            lastCols[0] = 0.01*resp_left + 0.99*lastCols[0];
    if RIGHT_LANE_DETECTION:
        if abs(lastCols[1] - resp_right) < 2:
            lastCols[1] = 0.2*resp_right + 0.8*lastCols[1];
        elif abs(lastCols[1] - resp_right) < 5:
            lastCols[1] = 0.15*resp_right + 0.85*lastCols[1];
        elif abs(lastCols[1] - resp_right) < 10:
            lastCols[1] = 0.15*resp_right + 0.85*lastCols[1];
        #elif abs(lastCols[1] - resp_right) < 20:
        else:
            lastCols[1] = 0.01*resp_right + 0.99*lastCols[1];
    
    # only consider detections around lastCol
    if LEFT_LANE_DETECTION:
        left_lane_min_x = lastCols[0]-max_lane_size;
        left_lane_max_x = lastCols[0]+max_lane_size;
    else:
        left_lane_min_x = midpoint_lastCols
        left_lane_max_x = midpoint_lastCols
    if RIGHT_LANE_DETECTION:
        right_lane_min_x = lastCols[1]-max_lane_size;
        right_lane_max_x = lastCols[1]+max_lane_size;
    else:
        right_lane_min_x = cols 
        right_lane_max_x = cols
   
    O[:,0:left_lane_min_x] = 0
    O[:,left_lane_max_x:right_lane_min_x] = 0
    O[:,right_lane_max_x:] = 0

    line_O = np.copy(O)
    #O[:,:,:] = 0 
    if lastCols[0] is not None:
        y, x = np.nonzero(line_O[:,0:midpoint_lastCols,0])
        if y.size > 20:
            #L = LinearLeastSquaresModel([0,1],[2]);
            #data = np.vstack([y, np.ones(len(x)), x]).transpose()
            #p_left = L.fit(data).reshape((2))
            #p_left = ransac(data,L,10,100,max_lane_size/5,15).reshape((2))

            p_left = np.polyfit(y, x, 1)
            #lastLine[0] = p_left[0]
            #lastLine[1] = p_left[1]
            lastLine[0] = 0.9*lastLine[0] + 0.1*p_left[0]
            lastLine[1] = 0.9*lastLine[1] + 0.1*p_left[1]
        for y in xrange(240):
            x = lastLine[0]*y + lastLine[1] 
            if x < 0 or x >= 320: continue
            O[y, lastLine[0]*y + lastLine[1]] = [255, 0, 0]

    if lastCols[1] is not None:
        y, x = np.nonzero(line_O[:, lastCols[1]-20:lastCols[1]+20,0])
        if y.size > 20:
            #L = LinearLeastSquaresModel([0,1],[2]);
            #data = np.vstack([y, np.ones(len(x)), x]).transpose()
            #p_right = ransac(data,L,10,100,max_lane_size/5,15).reshape((2))
            #lastLine[0] = p_right[0]
            #lastLine[2] = p_right[1]
            p_right = np.polyfit(y, x, 1)
            lastLine[2] = 0.9*lastLine[2] + 0.1*p_right[0]
            lastLine[3] = 0.9*lastLine[3] + 0.1*p_right[1]
        for y in xrange(240):
            x = lastLine[2]*y + lastLine[3] + lastCols[1] - 20
            if x < 0 or x >= 320: continue
            O[y, x] = [255, 0, 0]

    too_far = 9/16.0
    too_close = 6.5/16.0
    # if the cols want to move too close or too far, push them away/closer
    if lastCols[0] is not None and lastCols[1] is not None:
        if lastCols[1] - lastCols[0] > too_far*cols:
            lastCols[0] = midpoint_lastCols - too_far/2*cols;
            lastCols[1] = midpoint_lastCols + too_far/2*cols;
    
        if lastCols[1] - lastCols[0] < too_close*cols:
            lastCols[0] = midpoint_lastCols - too_close/2*cols;
            lastCols[1] = midpoint_lastCols + too_close/2*cols;

    """
    # output normalization
    O_min = np.amin(O)
    O_max = np.amax(O)
    O = (O - O_min) / (O_max - O_min)
    """
    O = cv2.warpPerspective(O, P, (cols, rows), flags=cv.CV_WARP_INVERSE_MAP)
    return (O, lastCols, lastLine)


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
