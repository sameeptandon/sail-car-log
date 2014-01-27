from LidarTransforms import * 
import sys, os
from VideoReader import *
from CameraParams import * 
from cv2 import imshow, waitKey
from numpy.linalg import norm
from ColorMap import *
from numpy import exp

if __name__ == '__main__': 
    video_filename = sys.argv[1] 
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    cam = getCameraParams()[cam_num - 1] 
    video_reader = VideoReader(video_filename,num_splits=1) # require only split_0 
    pts = loadPCD(sys.argv[2])
    width = 8
    # translate points in lidar frame to camera frame
    tx = -0.03
    ty = 0.1
    #ty = 0.0
    tz = -0.0
    count = 0 
    while count < 10:
        (success, I) = video_reader.getNextFrame()
        count += 1

    orig = I.copy()
    I = orig.copy()
    colormap = I.copy()
    colormap[:,:,:] = 0
    raw_pts = array(pts[:, 0:3])
    raw_pts[:, 0] += tx
    raw_pts[:, 1] += ty
    raw_pts[:, 2] += tz

    print 'load done'
    pts_wrt_cam = dot(R_to_c_from_l(cam), raw_pts.transpose())
    print pts_wrt_cam.shape

    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    mask = pix[0,:] > 0 + width/2
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    print mask.shape

    dist_wrt_cam = np.sum( pts[:, 0:3] ** 2, axis=1 ) ** (1.0/2)
    intensity = pts[:, 3]
    #colors = heatColorMapFast(dist_wrt_cam, 0, np.max(dist_wrt_cam))
    #colors = heatColorMapFast(exp(-dist_wrt_cam/3), 0, 1)
    colors = heatColorMapFast(intensity, 0, 255)

    px = pix[1,mask]
    py = pix[0,mask]
    I[px, py, :] = colors[0,mask,:]
    for p in range(-width/2,width/2):
        I[px+p,py, :] = colors[0,mask,:] 
        I[px,py+p, :] = colors[0,mask,:]
        I[px-p,py, :] = colors[0,mask,:]
        I[px,py-p, :] = colors[0,mask,:]
    imshow('display', I)
    waitKey()
