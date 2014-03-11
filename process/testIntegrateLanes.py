from Q50_config import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from VtkRenderer import *
from transformations import euler_matrix
import numpy as np
from ColorMap import *
import vtk
import copy
from CameraParams import * 
from ArgParser import *
import colorsys
from cv2 import imshow, waitKey
import cv2

#CAMERA 2


##################################
(rx,ry,rz) = (0,-0.015,-0.045)
R = euler_matrix(rx,ry,rz)[0:3,0:3]
T_from_l_to_i = np.eye(4)
T_from_l_to_i[0:3,0:3] = R.transpose()
T_from_i_to_l = np.linalg.inv(T_from_l_to_i)

# data = loadLDR(ldr_map[fnum])
# # filter out lanes
# #data = data[ data[:,3] > 60 ] 
# #data = data[ data[:,0] > 0 ]
# #data = data[ np.abs(data[:,1]) < 5]
# #data = data[ data[:,2] < -1.5]
# #data = data[ data[:,2] > -2.5]


# ###################################
(ctx, cty, ctz, crx, cry, crz) = \
        (-0.0, 0.31, 0.15, 0.049, 0.016, 0.014)
cR = euler_matrix(crx, cry, crz)[0:3,0:3]


# ###################################
# (success, I) = video_reader.getNextFrame()
# stepVideo(video_reader, step)

# pts_wrt_cam = array(data[:, 0:3])
# pts_wrt_cam[:, 0] += ctx
# pts_wrt_cam[:, 1] += cty
# pts_wrt_cam[:, 2] += ctz
# pts_wrt_cam = dot(cR, dot(R_to_c_from_l(cam), pts_wrt_cam.transpose()))
# (pix, mask) = cloudToPixels(cam, pts_wrt_cam)
start_fn = 0
num_fn = 10
step = 9

def stepVideo(video_reader, step):
    if step == 1: 
        return None
    for t in range(step-1):
        (success, I) = video_reader.getNextFrame()
    return success

def cloudToPixels(cam, pts_wrt_cam): 

    width = 4
    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    gps_filename = args['gps']
    
    cam = getCameraParams()[cam_num - 1] 
    video_reader = VideoReader(args['video'])
    gps_reader = GPSReader(gps_filename)
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    inv_imu_transforms = [np.linalg.inv(xform) for xform in imu_transforms]

    all_data = np.load(sys.argv[3])
    data = all_data['data']
    
    delta = 15
    
    count = 0
    while count < 400:
        video_reader.getNextFrame()
        stepVideo(video_reader, step)
        count += 1

    while True:
        pts = data[data[:, 4] < (video_reader.framenum + step * delta)]
        pts = pts[pts[:, 4] >= (video_reader.framenum)]
        #Lane filter

        (success, I) = video_reader.getNextFrame()
        stepVideo(video_reader, step)
        
        all_pix = None

        frame_num = video_reader.framenum - step
        # initial_rel_gps_inv = np.linalg.inv(imu_transforms[frame_num])
        for i in xrange(delta):
            cur_frame_num = i*step + frame_num

            cur_pts = pts[pts[:, 4] == cur_frame_num]
            # Convert to homogenous coordinates
            cur_pts_h = np.vstack((cur_pts.transpose()[:3, :], np.ones((1, cur_pts.shape[0]))))

            T_from_gps_to_l = dot(T_from_i_to_l, inv_imu_transforms[cur_frame_num])
            # Convert from point cloud to relative position of points from velodyne view        
            rel_cur_pts = dot(T_from_gps_to_l, cur_pts_h)
            rel_cur_pts = rel_cur_pts[:, (cur_pts[:, 3] > 60) &
                                         (rel_cur_pts[0, :] > 0) &
                                         (rel_cur_pts[2, :] < -1) &
                                         (abs(rel_cur_pts[1, :]) < 4)]
            # rel_cur_pts = rel_cur_pts[:, (cur_pts[:, 3] > 60)]
            if rel_cur_pts.shape[1] > 0:
                # Project the point cloud into the camera view
                pts_wrt_cam = rel_cur_pts[:3, :]
                pts_wrt_cam[0, :] += ctx
                pts_wrt_cam[1, :] += cty
                pts_wrt_cam[2, :] += ctz
                pts_wrt_cam = dot(cR, dot(R_to_c_from_l(cam), pts_wrt_cam))
                (pix, mask) = cloudToPixels(cam, pts_wrt_cam)
                
                pix = pix[:2, mask]
                intensity = cur_pts[mask, 3].astype(np.int32)
                # intensity = int(((i + 1) * 100 / delta)) * np.ones((1, pix.shape[1]))
                pix = np.vstack((pix, intensity)).astype(np.int32)
                all_pix = pix if all_pix == None else np.hstack((all_pix, pix))

        px = all_pix[1, :]
        py = all_pix[0, :]
        intensties = all_pix[2, :]

        colors = heatColorMapFast(intensties, 0, 100).astype(np.int32)[0, :, :]
        # color = colorsys.hsv_to_rgb(random.random(), 1, 1)
        # color = [int(c*255) for c in color]

        if px.shape > (0, 0):
            I[px, py, :] = colors
            for i in xrange(3):
                I[px+i,py, :] = colors
                I[px,py+i, :] = colors
                I[px-i,py, :] = colors
                I[px,py-i, :] = colors

        cv2.imshow('display', I)
        key = chr((waitKey(1) & 255))
        # while key != ' ':
        #     key = chr((waitKey(10000) & 255))

    # cloud = VtkPointCloud(data[:,0:3], data[:,3])
    # cloud_r = vtk.vtkRenderer()
    # cloud_r.SetBackground(0., 0., 0.)
    # cloud_r.SetViewport(0,0,1.0,1.0)
    # actor = cloud.get_vtk_cloud(zMin=0, zMax=255);
    # cloud_r.AddActor(actor)

    # # Render Window
    # renderWindow = vtk.vtkRenderWindow()
    # renderWindow.AddRenderer(cloud_r)
    # renderWindow.SetSize(1200, 600)

    # # Interactor
    # renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    # renderWindowInteractor.SetRenderWindow(renderWindow)
    # mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    # renderWindowInteractor.SetInteractorStyle(mouseInteractor)
    # renderWindow.Render()

    # renderWindowInteractor.Start()
