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

    pts_wrt_cam = dot(R_to_c_from_l(cam), raw_pts.transpose())
    print pts_wrt_cam.shape


    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    for idx in range(pix.shape[1]):
        if pix[0,idx] > 0 and pix[0,idx] < 1279-width/2 and pix[1,idx] > 0 and pix[1,idx] < 959-width/2 and pts_wrt_cam[2,idx] > 0:
            dist = norm(pts[idx,0:3])
            #print pts[idx,0:3]
            #print pts_wrt_cam[0:3,idx]
            #print dist
            intensity = pts[idx,3]
            print intensity, dist
            #rgbval = heatColorMap(intensity,0, 255)
            rgbval = heatColorMap(exp(-dist/3),0,1)
            print rgbval

            px = pix[1,idx]
            py = pix[0,idx]

            colormap[px,py,:] = rgbval
            I[px,py,:]=colormap[px,py,:]
            for p in range(-width/2,width/2):
                I[px+p,py, :] = colormap[px,py,:] 
                I[px,py+p, :] = colormap[px,py,:]
                I[px-p,py, :] = colormap[px,py,:]
                I[px,py-p, :] = colormap[px,py,:]


    #applyColorMap(colormap, cv2.COLORMAP_HOT) 
    imshow('display', I)
    waitKey()
