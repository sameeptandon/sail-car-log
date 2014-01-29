from LidarTransforms import * 
import sys, os
from VideoReader import *
from CameraParams import * 
from cv2 import imshow, waitKey
from numpy.linalg import norm
from ColorMap import *
from numpy import exp
from transformations import euler_matrix

if __name__ == '__main__': 
    video_filename = sys.argv[1] 
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    cam = getCameraParams()[cam_num - 1] 
    video_reader = VideoReader(video_filename,num_splits=1) # require only split_0 
    pts = loadLDR(sys.argv[2])

    width = 1
    # translate points in lidar frame to camera frame
    tx = -0.00
    ty = 0.1
    tz = -0.0
    rx = 0.0
    ry = 0.0
    rz = 0.0
    count = 0 
    while count < 10:
        (success, I) = video_reader.getNextFrame()
        count += 1

    orig = I.copy()
    while True:
        I = orig.copy()
        raw_pts = array(pts[:, 0:3])
        raw_pts[:, 0] += tx
        raw_pts[:, 1] += ty
        raw_pts[:, 2] += tz
        R = euler_matrix(rx, ry, rz)[0:3,0:3]


        pts_wrt_cam = dot(R, dot(R_to_c_from_l(cam), raw_pts.transpose()))

        pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
        pix = pix.astype(np.int32)
        mask = np.logical_and(True, pix[0,:] > 0 + width/2)
        mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
        mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
        mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
        mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)

        dist_wrt_cam = np.sum( pts[mask, 0:3] ** 2, axis=1 ) ** (1.0/2)
        intensity = pts[mask, 3]
        #colors = heatColorMapFast(dist_wrt_cam, 0, np.max(dist_wrt_cam))
        #colors = heatColorMapFast(exp(-dist_wrt_cam/3), 0, 1)
        colors = heatColorMapFast(intensity, 0, 255)

        px = pix[1,mask]
        py = pix[0,mask]
        I[px, py, :] = colors[0,:,:]
        """
        for p in range(-width/2,width/2):
            I[px+p,py, :] = colors[0,:,:] 
            I[px,py+p, :] = colors[0,:,:]
            I[px-p,py, :] = colors[0,:,:]
            I[px,py-p, :] = colors[0,:,:]
        """
        imshow('display', I)
        key = chr((waitKey() & 255))
        
        if key == 'w':
            tx += 0.001
        elif key == 'e':
            tx -= 0.001
        elif key == 'r':
            ty += 0.001
        elif key == 't':
            ty -= 0.001
        elif key == 'y':
            tz += 0.001
        elif key == 'u':
            tz -= 0.001
        elif key == 'a':
            rx += 0.001
        elif key == 's':
            rx -= 0.001
        elif key == 'd':
            ry += 0.001
        elif key == 'f':
            ry -= 0.001
        elif key == 'g':
            rz += 0.001
        elif key == 'h':
            rz -= 0.001
        

        print (tx, ty, tz, rx, ry, rz)
