from Q50_config import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
import numpy as np
import cv2
from ArgParser import *
import vtk
from VtkRenderer import *
import multiprocessing

NUM_PROCESSES = 4
WINDOW = 50*3

def cloudToPixels(cam, pts_wrt_cam): 

    mask = np.logical_and(True,pts_wrt_cam[2,:] > 0)
    width = 4
    (rpix, J)  = cv2.projectPoints(pts_wrt_cam[:,mask].transpose(), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), cam['KK'], cam['distort'])
    rpix = rpix.transpose()
    rpix = np.around(rpix[:, 0, :])
    
    pix = np.zeros((2,mask.shape[0]))
    pix[:,mask] = rpix;
    pix = pix.astype(np.int32)

    mask = np.logical_and(mask, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)

    return (pix, mask)

def localMapToPixels(map_data, imu_transforms_t, T_from_i_to_l, cam):
    # load nearby map frames
    pts_wrt_imu_0 = array(map_data[:,0:3]).transpose()
    pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0, 
        np.ones((1,pts_wrt_imu_0.shape[1]))))
    # transform points from imu_0 to imu_t
    pts_wrt_imu_t = np.dot( np.linalg.inv(imu_transforms_t), pts_wrt_imu_0)
    # transform points from imu_t to lidar_t
    pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t);
    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = dot(R_to_c_from_l(cam), 
            pts_wrt_camera_t.transpose())
    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    return (pix, mask)

def getPixelColors(args):
    (I, map_data, t) = args
    mask_window = (map_data[:,4] < t + WINDOW) & (map_data[:,4] > t );

    # reproject
    (pix, mask) = localMapToPixels(map_data[mask_window,:], imu_transforms[t,:,:], T_from_i_to_l, cam);

    color_data = np.zeros((np.count_nonzero(mask), 3))

    color_data[:,0] = I[pix[1,mask], pix[0,mask], 2]
    color_data[:,1] = I[pix[1,mask], pix[0,mask], 1]
    color_data[:,2] = I[pix[1,mask], pix[0,mask], 0]

    return (color_data,mask)



if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])

    params = args['params']
    cam = params['cam'][cam_num - 1]
    video_reader = VideoReader(args['video'])
    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    
    T_from_i_to_l = np.linalg.inv(params['lidar']['T_from_l_to_i'])

    all_data = np.load(sys.argv[3])
    map_data = all_data['data']
    color_data = np.zeros((map_data.shape[0], 3))
    #color_data = np.zeros((map_data.shape[0], 7, 10))
    #map_data = map_data[map_data[:,3] > 60, :]
   
    start_t = 0
    t = start_t
    while True:
        video_reader.setFrame(t+25)
        (success, I) = video_reader.getNextFrame()

        t = video_reader.framenum - 1
        print t
        if t > np.max(map_data[:,4]):
            break


        mask_window = (map_data[:,4] < t + WINDOW) & (map_data[:,4] > t );

        # reproject
        #(pix, mask) = localMapToPixels(map_data[mask_window,:], imu_transforms[t,:,:], T_from_i_to_l, cam);
        (cdata, mask) = getPixelColors((I, map_data, t)) 


        mask_window[mask_window] &= mask
        color_data[mask_window, :] = cdata
        #color_data[mask_window,0] = I[pix[1,mask], pix[0,mask], 2]
        #color_data[mask_window,1] = I[pix[1,mask], pix[0,mask], 1]
        #color_data[mask_window,2] = I[pix[1,mask], pix[0,mask], 0]
        """
        idx_num = color_data[mask_window,6,0].astype(np.int32)
        color_data[mask_window,0,idx_num] = I[pix[1,mask], pix[0,mask], 2]
        color_data[mask_window,1,idx_num] = I[pix[1,mask], pix[0,mask], 1]
        color_data[mask_window,2,idx_num] = I[pix[1,mask], pix[0,mask], 0]
        color_data[mask_window,6,0] += 1
        
        """
        """
        color_data[mask_window,3] = np.divide(I[pix[1,mask],pix[0,mask],2] + (color_data[mask_window,6]-1)*color_data[mask_window,3], color_data[mask_window,6])

        color_data[mask_window,4] = np.divide(I[pix[1,mask],pix[0,mask],1] + (color_data[mask_window,6]-1)*color_data[mask_window,4], color_data[mask_window,6])

        color_data[mask_window,5] = np.divide(I[pix[1,mask],pix[0,mask],0] + (color_data[mask_window,6]-1)*color_data[mask_window,5], color_data[mask_window,6])
        """

        """
        color_data[mask_window,0] = I[pix[1,mask],pix[0,mask],2]
        color_data[mask_window,1] = I[pix[1,mask],pix[0,mask],1]
        color_data[mask_window,2] = I[pix[1,mask],pix[0,mask],0]
        """
    
        """
        # draw
        intensity = map_data_copy[mask, 3]
        heat_colors = heatColorMapFast(intensity, 0, 100)
        for p in range(4):
            I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]
            I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]

        cv2.imshow('vid', cv2.pyrDown(I))
        cv2.waitKey(1)
        """

    #print np.max(color_data[:,6])
    """
    vtk_color_data = np.zeros((map_data.shape[0], 3))
    for j in range(int(np.max(color_data[:,6]) + 1)):
        med_mask = (color_data[:,6,0] == j)
        if j == 0:
            vtk_color_data[med_mask, :] = 0
        else:   
            vtk_color_data[med_mask, :] = np.std(color_data[med_mask,0:3,0:j], axis=2)
    """

    time_mask = map_data[:,4] >= start_t
    vtk_color_data = color_data[:,0:3]
    
    lidarCloud = VtkPointCloud(map_data[time_mask,0:3], vtk_color_data[time_mask, 0:3])
    #tot_std_dev = np.linalg.norm(vtk_color_data[time_mask,0:3],axis=1)
    #print tot_std_dev.shape
    #lidarCloud = VtkPointCloud(map_data[time_mask,0:3], tot_std_dev)

    cloud_r = vtk.vtkRenderer()
    cloud_r.SetBackground(0., 0., 0.)
    cloud_r.SetViewport(0,0,1.0,1.0)
    cloud_r.AddActor(lidarCloud.get_vtk_color_cloud())
    #cloud_r.AddActor(lidarCloud.get_vtk_cloud(zMin=0, zMax=np.max(tot_std_dev)))
    #cloud_r.AddActor(lidarCloud.get_vtk_cloud(zMin=0, zMax=255))
    cloud_r.ResetCamera()
    # Render Window
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(cloud_r)
    renderWindow.SetSize(1200, 600)

    # Interactor
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    renderWindowInteractor.SetInteractorStyle(mouseInteractor)
    renderWindow.Render()

    #renderWindowInteractor.AddObserver('KeyPressEvent', keypress)
    renderWindowInteractor.Start()
