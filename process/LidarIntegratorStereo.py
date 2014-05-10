# Stereo Integrator
#
# usage: 
# python LidarIntegrator.py <dir> <basename><camnum>.avi <export name>.npz <optional additional flags such as --export or --full>

# to change the type of data exported, see the function integrateClouds 


from Q50_config import *
from ArgParser import *
import sys, os
from GPSReader import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from VtkRenderer import *
from transformations import euler_matrix
import numpy as np
from ColorMap import *
import vtk
import copy
import pdb
import cv2
from StereoCompute import *

global actors
global clouds 
global all_data
global cloud_r
global renderWindow
global rx
global ry
global rz
global R
global start_fn
global num_fn
global color_mode
global exportLidar
global exportStereo

actors =  []
clouds = [ ]
all_data = [ ] 
cloud_r = vtk.vtkRenderer()
renderWindow = vtk.vtkRenderWindow()

#(rx,ry,rz) = (-0.1)
#R_from_i_to_l = euler_matrix(rx,ry,rz)[0:3,0:3]

# settings for 280N_e 
#start_fn = 1200 
#num_fn = 1000 
#step = 1

# settings for vail 
#start_fn = 0
#num_fn = 300
#step = 5

# settings for parking
#start_fn = 0
#num_fn = 200
#step = 2



start_fn = 0 # offset in frame numbers to start exporting data
num_fn = 60 # number of frames to export. this is changed if --full is enabled; how many lidar frames to export
step = 10 # step between frames
color_mode = 'INTENSITY'
exportStereo = False
exportLidar = False

def exportData():
        print 'exporting data'
        export_data = np.row_stack(all_data)
        print export_data
        print export_data.shape
        np.savez(sys.argv[3], data=export_data)
        print 'export complete'

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

def stepVideo(video_reader, step):
    if step == 1: 
        return None
    for t in range(step-1):
        (success, I) = video_reader.getNextFrame()
    return success

def integrateClouds(ldr_map, stereo_map, IMUTransforms, renderer, offset, num_steps, step, calibrationParameters):
   
    start = offset
    end = offset + num_steps*step

    trans_wrt_imu = IMUTransforms[start:end,0:3,3]
    gpsPointCloud = VtkPointCloud(trans_wrt_imu[:,0:3], 0*trans_wrt_imu[:,0])
    clouds.append(gpsPointCloud)
    actors.append(gpsPointCloud.get_vtk_cloud())
    renderer.AddActor(actors[-1])

    for t in range(num_steps):
        fnum = offset+t*step
        print fnum

	#stereo mapping

        stereo_data = stereo_map[t]

	pts_stereo = stereo_data[:,0:3].transpose()
        pts_stereo = np.vstack((pts_stereo,np.ones((1,pts_stereo.shape[1]))))
        T_from_l_to_i = calibrationParameters['lidar']['T_from_l_to_i'] 
        pts_stereo = np.dot(T_from_l_to_i, pts_stereo)
        pts_stereo = np.dot(IMUTransforms[fnum,:,:], pts_stereo);
        pts_stereo = pts_stereo.transpose()


	if exportStereo == True:
#		pdb.set_trace()
                pts_stereo_copy = array(pts_stereo[:,0:3])
                pts_stereo_copy = np.column_stack((pts_stereo_copy, -10 + 0*array(stereo_data[:,2])))
                pts_stereo_copy = np.column_stack((pts_stereo_copy, fnum*np.ones((pts_stereo.shape[0],1))))
                all_data.append(pts_stereo_copy)


        if color_mode == 'CAMERA':
            stereoCloud = VtkPointCloud(pts_stereo[ mask2 ,0:3], colors[ mask2,:])
            actors.append(stereoCloud.get_vtk_color_cloud())
        elif color_mode == 'INTENSITY':
            stereoCloud = VtkPointCloud(pts_stereo[:,0:3], -10 + 0*stereo_data[:,2])
            actors.append(stereoCloud.get_vtk_cloud(zMin=-10, zMax=10))

        clouds.append(stereoCloud)
        renderer.AddActor(actors[-1])

	#lidar mapping
        
	lidar_data = loadLDR(ldr_map[fnum])
	dist = np.sqrt(np.sum( lidar_data[:, 0:3] ** 2, axis = 1))

        # check out the commented out section below to figure out how this is filtering.                                                   
        lidar_data_filter_mask = (dist > 3)                 & \
                           (lidar_data[:,3] > 30)
                           #&\                                                                                                             
                           #(np.abs(data[:,1]) < 2.2)   & \                                                                                
                           #(np.abs(data[:,1]) > 1.2)   & \                                                                                
                           #(data[:,2] < -1.8)          & \                                                                                
                           #(data[:,2] > -2.5)                                                                                             
        lidar_data = lidar_data[lidar_data_filter_mask, :]


	pts_lidar = lidar_data[:,0:3].transpose()
        pts_lidar = np.vstack((pts_lidar,np.ones((1,pts_lidar.shape[1]))))
        T_from_l_to_i = calibrationParameters['lidar']['T_from_l_to_i'] #transform from lidar to imu                                       
        pts_lidar = np.dot(T_from_l_to_i, pts_lidar)
        # transform data into imu_0 frame                                                                                                  
        pts_lidar = np.dot(IMUTransforms[fnum,:,:], pts_lidar); #Tx4x4. T is timesteps, and 4x4 matrix is rotation/translation of IMU from time0       
        pts_lidar = pts_lidar.transpose()


	if exportLidar == True:
#		pdb.set_trace()
		pts_lidar_copy = array(pts_lidar[:,0:3])
		pts_lidar_copy = np.column_stack((pts_lidar_copy, array(lidar_data[:,3])))
		pts_lidar_copy = np.column_stack((pts_lidar_copy, fnum*np.ones((pts_lidar.shape[0],1))))
		all_data.append(pts_lidar_copy)

        if color_mode == 'CAMERA':
            lidarCloud = VtkPointCloud(pts_lidar[ mask2 ,0:3], colors[ mask2,:])
            actors.append(lidarCloud.get_vtk_color_cloud())
        elif color_mode == 'INTENSITY':
            lidarCloud = VtkPointCloud(pts_lidar[:,0:3], lidar_data[:,3])
            actors.append(lidarCloud.get_vtk_cloud(zMin=0, zMax=255))
        clouds.append(lidarCloud)
        renderer.AddActor(actors[-1])


def keypress(obj, event):
    global actors
    global clouds 
    global cloud_r
    global all_data
    global renderWindow
    global rx
    global ry
    global rz
    global R
    global start_fn
    global color_mode
    key = obj.GetKeySym()
    rerender = True
    if key == 'i':
        ry += 0.0005
    elif key == 'k':
        ry -= 0.0005
    elif key == 'l':
        rx += 0.005
    elif key == 'j':
        rx -= 0.005
    elif key == 'o':
        rz += 0.0005
    elif key == 'u':
        rz -= 0.0005
    elif key == 'c':
        color_mode = 'CAMERA'
    elif key == 'v':
        color_mode = 'INTENSITY'
    elif key == 'x':
        exportData()

    else:
        rerender = False
    if rerender:
        for a in actors:
            cloud_r.RemoveActor(a)
        actors = [ ]
        clouds = [ ]
        all_data = [ ]
        #start_fn = start_fn + 5
        integrateClouds(ldr_map, imu_transforms, cloud_r, start_fn, num_fn, step, params)
        renderWindow.Render()
    print key
    #print (rx,ry,rz)

if __name__ == '__main__': 
    vfname = sys.argv[2]
    vidname = vfname.split('.')[0]
    vidname2 = vidname[:-1] + '2'
    video_filename2 = sys.argv[1] + '/' + vidname2 + '.avi'
    
    args = parse_args(sys.argv[1], sys.argv[2])

    gps_reader = GPSReader(args['gps'])
    params = args['params']
    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    ldr_map = loadLDRCamMap(args['map'])

    if '--full' in sys.argv:
	    total_num_frames = GPSData.shape[0]
	    start_fn = 0
	    step = 10
	    num_fn = int(total_num_frames / step)

    #Loading Stereo

    stereo_map = []
    reader_left = VideoReader(args['video'])
    reader_right = VideoReader(args['opposite_video'])
    finished = False

    reader_left.setFrame(start_fn)
    reader_right.setFrame(start_fn)
    (success, imgL) = reader_left.getNextFrame()
    (success, imgR) = reader_right.getNextFrame()

    while finished == False:

	 (disp, Q, R1, R2) = siftStereo(imgL, imgR, params)
	 #cv2.imshow('disp', disp)
         #print Q
	 stereo_points = get3dPoints(disp,Q)
	 stereo_points = stereo_points[disp > 5, :]
         #print stereo_points
	 stereo_points = stereo_points.transpose()
	 stereo_points = np.dot(R1.transpose(), stereo_points)
         #print np.amax(stereo_points, axis=1)
         #print np.amin(stereo_points, axis=1)
	 stereo_points = np.vstack((stereo_points, np.ones((1,stereo_points.shape[1])))) 
         #by here, you have 3D stereo points wrt camera                                       
         #print stereo_points.shape
	 stereo_points = dot(np.linalg.inv(params['cam'][0]['E']), stereo_points)
	 stereo_wrt_lidar = np.dot(R_to_c_from_l(params['cam'][0]).transpose(), stereo_points[0:3,:])
	 stereo_wrt_lidar = stereo_wrt_lidar.transpose()
	 stereo_wrt_lidar = stereo_wrt_lidar[:,0:3] - params['cam'][0]['displacement_from_l_to_c_in_lidar_frame']
         #by here, same camera points but to lidar frame at time t                                                                      
	 #img = cv2.resize(img, (640, 480))                                                                                             
	 imgL = cv2.pyrDown(imgL)
         #cv2.imshow('disparity', cv2.pyrDown(disp)/64.0)                                                                               
	 stereo_map.append(stereo_wrt_lidar)

	 framenum = reader_left.framenum
	 print framenum

	 if framenum >= start_fn + num_fn*step:
		 finished = True
		 break
	 
	 for t in range(step):
		 (success, imgL) = reader_left.getNextFrame()
		 (success, imgR) = reader_right.getNextFrame()
	 
	 if success == False:
		 finished = True
		 break

    # this has been flipped for the q50
    
    cloud_r.SetBackground(0., 0., 0.)
    cloud_r.SetViewport(0,0,1.0,1.0)

    if '--lidar' in sys.argv:
	    exportLidar = True
	    exportStereo = False

    if '--stereo' in sys.argv:
	    exportStereo = True
	    exportLidar = False

    integrateClouds(ldr_map, stereo_map, imu_transforms, cloud_r, start_fn, num_fn, step, params)

    if '--export' in sys.argv:
      exportData()
      sys.exit(0)


    # Render Window
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(cloud_r)
    renderWindow.SetSize(1200, 600)

    axisActor = vtk.vtkAxisActor()
    axisActor.SetGridlineXLength(5)
    axisActor.SetGridlineYLength(5)
    axisActor.SetGridlineZLength(5)
    #axisActor.DrawGridpolysOn()
    #axisActor.DrawInnerGridlinesOn()
    cloud_r.AddActor(axisActor)

    # Interactor
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    renderWindowInteractor.SetInteractorStyle(mouseInteractor)
    renderWindow.Render()

    renderWindowInteractor.AddObserver('KeyPressEvent', keypress)
    #renderWindowInteractor.AddObserver('TimerEvent', keypress)
    #timerId = renderWindowInteractor.CreateRepeatingTimer(1)
    renderWindowInteractor.Start()
