from LidarTransforms import * 
import sys, os
from VideoReader import *
from CameraParams import * 
from cv2 import imshow, waitKey
import cv2
from numpy.linalg import norm
from ColorMap import *
from numpy import exp
from transformations import euler_matrix

def getNextData(VideoReader, LDRFrameMap):
    for idx in range(5):
        (success, img) = VideoReader.getNextFrame()
        if not success:
            return None
    ldr_frame = loadLDR(LDRFrameMap[VideoReader.framenum])
    return (success, img, ldr_frame)

def cloudToPixels(cam, pts_wrt_cam): 

    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3,:] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

if __name__ == '__main__': 
    video_filename = sys.argv[1] 
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    cam = getCameraParams()[cam_num - 1] 
    video_reader = VideoReader(video_filename,num_splits=1) # require only split_0 

    width = 9
    # translate points in lidar frame to camera frame

    #(-0.20000000000000007, 0.4, 0.32999999999999985, -0.09800000000000006, 0.014999999999999986, -0.02400000000000002)
    #(-0.10000000000000007, 0.5, 0.4299999999999998, -0.09800000000000006, 0.019999999999999987, 0.000999999999999982)
    (tx, ty, tz, rx, ry, rz) = \
            (-0.0, 0.30999999999999994, 0.15, 0.04899999999999999, 0.015999999999999997, 0.014000000000000005)
            #(-0.0, 0.35, 0.15, 0.05499999999999999, 0.02, 0.0)
            #[-0.30493086,  0.41796525,  0.39775339, -0.0986506,   0.01598486,  0.00963721]

            #(-0.30000000000000004, 0.5, 0.4299999999999998, -0.09800000000000006, 0.019999999999999987, 0.010999999999999982)
    video_reader = VideoReader(video_filename)
    ldr_frame_map = loadLDRCamMap(sys.argv[2])

    while True:

        (success, I, pts) = getNextData(video_reader, ldr_frame_map)
        orig = I.copy()
        I = orig.copy()
        if not success: 
            break
        #pts = pts[pts[:,3] > 40]
        #pts = pts[pts[:,0] > 0,:]
        #pts = pts[pts[:,2] < -0.7,:]
        raw_pts = array(pts[:, 0:3])
        raw_pts[:, 0] += tx
        raw_pts[:, 1] += ty
        raw_pts[:, 2] += tz
        R = euler_matrix(rx, ry, rz)[0:3,0:3]


        pts_wrt_cam = dot(R, dot(R_to_c_from_l(cam), raw_pts.transpose()))
        (pix, mask) = cloudToPixels(cam, pts_wrt_cam)

        #dist_wrt_cam = np.sum( pts[mask, 0:3] ** 2, axis=1 ) ** (1.0/2)
        intensity = pts[mask, 3]
        
        #colors = heatColorMapFast(dist_wrt_cam, 0, np.max(dist_wrt_cam))
        #colors = heatColorMapFast(exp(-dist_wrt_cam/3), 0, 1)
        colors = heatColorMapFast(intensity, 0, 100)

        px = pix[1,mask]
        py = pix[0,mask]
        I[px, py, :] = colors[0,:,:]
        for p in range(0,width/2):
            I[px+p,py, :] = colors[0,:,:] 
            I[px,py+p, :] = colors[0,:,:]
            I[px-p,py, :] = colors[0,:,:]
            I[px,py-p, :] = colors[0,:,:]
        imshow('display', I)
        key = chr((waitKey(1) & 255))
        if key == '+':
            tx += 0.005
        elif key == '_':
            tx -= 0.005
        elif key == 'a':
            ty += 0.005
        elif key == 'd':
            ty -= 0.005
        elif key == 'w':
            tz += 0.005
        elif key == 's':
            tz -= 0.005
        
        elif key == 'i':
            rx += 0.0005
        elif key == 'k':
            rx -= 0.0005
        elif key == 'l':
            ry += 0.0005
        elif key == 'j':
            ry -= 0.0005
        elif key == 'o':
            rz += 0.0005
        elif key == 'u':
            rz -= 0.0005

        print (tx, ty, tz, rx, ry, rz)
