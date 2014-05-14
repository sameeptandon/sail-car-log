import numpy as np
import skimage.feature
import sys
from VideoReader import *
from scipy.ndimage.filters import convolve
from scipy.io import loadmat
import cv
import cv2


def findLanesConvolution(img, origSize=(960,1280), lastCols=[None, None], lastLine=[None,None,None,None], P=np.eye(3), responseOnlyNearLastCols=False, frame_rate=1):
    (rows, cols, channels) = img.shape
    border_img = np.zeros((rows,cols,3))
    border_img = border_img.astype(np.uint8)
    border_img[:,:15] = 255
    border_img[:,-15:] = 255
    border_img[-15:,:] = 255
    
    img = cv2.warpPerspective(img, P, (cols, rows))
    img = img.astype(np.float64)
    border_img = cv2.warpPerspective(border_img, P, (cols, rows))

    """
    #idea for windowing around current lastCol, but could be bad as possible
    #to not recover on a bad misdetection
    if responseOnlyNearLastCols == True:
        left_lane_min_x = max(0,lastCols[0]-40);
        left_lane_max_x = lastCols[0]+40;
        right_lane_min_x = lastCols[1]-40;
        right_lane_max_x = min(cols,lastCols[1]+40);
        img[:,0:left_lane_min_x] = 0
        img[:,left_lane_max_x:right_lane_min_x] = 0
        img[:,right_lane_max_x:] = 0
        m_left = np.mean(np.mean(img[:,left_lane_min_x : left_lane_max_x],axis=0),axis=0)
        m_right = np.mean(np.mean(img[:,right_lane_min_x : right_lane_max_x],axis=0),axis=0)
        print m_left
        print np.max(img[:,:,0])
        img[:, left_lane_min_x : left_lane_max_x,:] -= m_left
        img[:, right_lane_min_x : right_lane_max_x,:] -= m_right
        print np.max(img[:,:,0])
    else:  
    """
    # mean subtraction on image, clamp to 0
    m = np.mean(np.mean(img[:,:],axis=0),axis=0)
    img = img - m
    img[img < 0] = 0
    img = 255 * img / np.max(img)

    # set max_lane_size to about 20 in the 1280x960 image
    max_lane_size = int(np.round(origSize[1] / 80)) # approximation of lane width
    if max_lane_size % 2 == 1:
        max_lane_size += 1

    O = np.zeros((rows, cols, channels))
    lane_width = max_lane_size
    v = 4*np.array([np.concatenate([-1*np.ones(lane_width), 2*np.ones(lane_width+1), -1*np.ones(lane_width)])])
    v = v/v.size
    O_1 = np.round(convolve(img[:,:,0], v, mode='reflect')).reshape((rows,cols,1)) 
    O_2 = np.round(convolve(img[:,:,1], v, mode='reflect')).reshape((rows,cols,1)) 
    O_3 = np.round(convolve(img[:,:,2], v, mode='reflect')).reshape((rows,cols,1)) 

    O = cv2.merge([O_1, O_2, O_3])
   
    # get rid of perspective transform border detections 
    O[border_img > 0] = 0 

    # thresholding for lane detection
    #white_lane_detect = np.sum(O,axis=2) > 350
    white_lane_detect = np.logical_and(O[:,:,0] > 150, np.logical_and(O[:,:,1] > 150, O[:,:,2] > 150))
    #yellow_lane_detect = np.logical_and(O[:,:,1] + O[:,:,2] > 90, O[:,:,0] < 20)
    eps = 0.000001
    yellow_lane_detect = np.logical_and(((O[:,:,1] + O[:,:,2]) / (eps + O[:,:,0]) ) > 5, O[:,:,1] + O[:,:,2] > 150) 
    low_vals = np.logical_and(np.logical_not(white_lane_detect), np.logical_not(yellow_lane_detect))
    O[low_vals,:] = 0

    # increase yellow lane detection score
    O[yellow_lane_detect,:] *= 5

    column_O = np.sum(np.sum(O,axis=2),axis=0);
    column_O[column_O < 1000] = 0
    O[:,column_O < 1000,:] = 0

    #O = cv2.warpPerspective(O, P, (cols, rows), flags=cv.CV_WARP_INVERSE_MAP)
    return (O, lastCols, lastLine)

def findLanes(O, origSize=(960,1280), lastCols=[None, None], lastLine=[None,None,None,None], P=np.eye(3), responseOnlyNearLastCols=False, frame_rate=1):
  
    O = O.astype(np.float64)
    (rows, cols, channels) = O.shape
    
    max_lane_size = int(np.round(origSize[1] / 96)) # approximation of lane width
    if max_lane_size % 2 == 1:
        max_lane_size += 1

    #O = cv2.warpPerspective(O, P, (cols, rows))
    """
    #mean subtract output image
    m = np.mean(np.mean(O[rows/2:rows,:],axis=0),axis=0)
    O = O - m
    O[O < 0] = 0
    """


    # get rid of top of the image so we don't find a column fit to it
    O[0:0*rows/4,:] = 0

    # nd the midpoint and submidpoints for 
    # zero-ing the center of the lane
    midpoint_lastCols = 0.5*(lastCols[0] + lastCols[1])
    mid_left = 0.5*(lastCols[0] + midpoint_lastCols);
    mid_right = 0.5*(lastCols[1] + midpoint_lastCols);
    O[:, mid_left:mid_right,:] = 0

    # compute the sum of activations in each column and find the max 
    # responding column on the left and right sides
    column_O = np.sum(np.sum(O,axis=2),axis=0);
    column_O[column_O < 8000] = 0
    O[:,column_O < 8000,:] = 0
    top_k = 15
    top_k = min(top_k, np.nonzero(column_O[0:midpoint_lastCols])[0].size)
    top_k = min(top_k, np.nonzero(column_O[midpoint_lastCols:])[0].size)

    resp_left = np.copy(column_O[0:midpoint_lastCols].argsort()[-top_k:][::-1])
    min_left = np.argmax(np.abs(resp_left)) # closest to midpoint
    resp_left = resp_left[min_left]

    resp_right = np.copy(column_O[midpoint_lastCols:].argsort()[-top_k:][::-1])
    min_right = np.argmin(np.abs(resp_right)) #closest to midpoint
    resp_right = resp_right[min_right] + midpoint_lastCols

    # was there a detection on either side? 
    LEFT_LANE_DETECTION = np.max(column_O[0: midpoint_lastCols]) != 0
    RIGHT_LANE_DETECTION = np.max(column_O[midpoint_lastCols:]) != 0

    if not LEFT_LANE_DETECTION:
        resp_right = lastCols[1]
    if not RIGHT_LANE_DETECTION:
        resp_left = lastCols[0]

    # how to move the columns based on previous cols 
    left_mom = 0.05
    if LEFT_LANE_DETECTION:
        if abs(lastCols[0] - resp_left) < 2:
            left_mom = 0.5
        elif abs(lastCols[0] - resp_left) < 5:
            left_mom = 0.3
        elif abs(lastCols[0] - resp_left) < 10:
            left_mom = 0.15
        left_mom = 1 - (1 - left_mom) ** frame_rate
        lastCols[0] = left_mom*resp_left + (1 - left_mom)*lastCols[0]
    right_mom = 0.05
    if RIGHT_LANE_DETECTION:
        if abs(lastCols[1] - resp_right) < 2:
            right_mom = 0.5
        elif abs(lastCols[1] - resp_right) < 5:
            right_mom = 0.3
        elif abs(lastCols[1] - resp_right) < 10:
            right_mom = 0.15
        right_mom = 1 - (1 - right_mom) ** frame_rate
        lastCols[1] = right_mom*resp_right + (1 - right_mom)*lastCols[1]
    
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

    
    #too_far = 9/16.0
    #too_close = 6.5/16.0
    too_far = 5.5/16.0
    too_close = 4.5/16.0
    # if the cols want to move too close or too far, push them away/closer
    if lastCols[0] is not None and lastCols[1] is not None:
        if lastCols[1] - lastCols[0] > too_far*cols:
            if LEFT_LANE_DETECTION:
                lastCols[0] = midpoint_lastCols - too_far/2*cols;
            if RIGHT_LANE_DETECTION:
                lastCols[1] = midpoint_lastCols + too_far/2*cols;
    
        if lastCols[1] - lastCols[0] < too_close*cols:
            if midpoint_lastCols < too_close * cols / 2:
                midpoint_lastCols = too_close * cols / 2
            if midpoint_lastCols > cols - too_close * cols / 2:
                midpoint_lastCols = 1 - too_close * cols / 2
            if LEFT_LANE_DETECTION:
                lastCols[0] = midpoint_lastCols - too_close/2*cols;
            if RIGHT_LANE_DETECTION:
                lastCols[1] = midpoint_lastCols + too_close/2*cols;

    O = cv2.warpPerspective(O, P, (cols, rows), flags=cv.CV_WARP_INVERSE_MAP)
    return (O, lastCols, lastLine)

