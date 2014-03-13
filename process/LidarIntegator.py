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
import cv2


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

actors =  []
clouds = [ ]
all_data = [ ] 
cloud_r = vtk.vtkRenderer()
renderWindow = vtk.vtkRenderWindow()
(rx,ry,rz) = (0,-0.015,-0.045)
R = euler_matrix(rx,ry,rz)[0:3,0:3]

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

start_fn = 0
num_fn = 10
step = 9

color_mode = 'INTENSITY'

(ctx, cty, ctz, crx, cry, crz) = \
        (-0.0, 0.30999999999999994, 0.15, 0.04899999999999999, 0.015999999999999997, 0.014000000000000005)
cR = euler_matrix(crx, cry, crz)[0:3,0:3]

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

def loadClouds(ldr_map, offset, step, num_steps):
    all_data  = [ ]
    for t in range(num_steps):
        data = loadLDR(ldr_map[offset + t*step])
        
        # filter out the roof rack
        dist = np.sqrt(np.sum( data[:, 0:3] ** 2, axis = 1))
        data = data[ dist > 3, :]

        all_data.append(data)
    return all_data


def stepVideo(video_reader, step):
    if step == 1: 
        return None
    for t in range(step-1):
        (success, I) = video_reader.getNextFrame()
    return success

def loadImage(video_reader, framenum):
    video_reader.setFrame(framenum)
    (success, I) = video_reader.getNextFrame()
    return (success, I)

def integrateClouds(ldr_map, IMUTransforms, renderer, R, offset, num_steps, step):
    start = offset
    end = offset + num_steps*step

    video_reader.setFrame(start)

    T_from_l_to_i = np.eye(4)
    T_from_l_to_i[0:3,0:3] = R.transpose()
    
    trans_wrt_imu = IMUTransforms[start:end,0:3,3]
    #trans_wrt_lidar = np.dot(R, trans_wrt_imu.transpose())
    #trans_wrt_lidar = trans_wrt_lidar.transpose()
    gpsPointCloud = VtkPointCloud(trans_wrt_imu[:,0:3], 0*trans_wrt_imu[:,0])
    clouds.append(gpsPointCloud)
    actors.append(gpsPointCloud.get_vtk_cloud())
    renderer.AddActor(actors[-1])
    for t in range(num_steps):
        fnum = offset+t*step

        
        data = loadLDR(ldr_map[fnum])
        # filter out the roof rack
        dist = np.sqrt(np.sum( data[:, 0:3] ** 2, axis = 1))
        data = data[ dist > 3, :]
        #data = data[ data[:,3] > 60 ] 
        #data = data[ data[:,0] > 0 ]
        #data = data[ np.abs(data[:,1]) < 5]
        #data = data[ data[:,2] < -1.5]
        #data = data[ data[:,2] > -2.5]
        
        (success, I) = video_reader.getNextFrame()
        stepVideo(video_reader, step)
        
        pts_wrt_cam = array(data[:, 0:3])
        pts_wrt_cam[:, 0] += ctx
        pts_wrt_cam[:, 1] += cty
        pts_wrt_cam[:, 2] += ctz
        pts_wrt_cam = dot(cR, dot(R_to_c_from_l(cam), pts_wrt_cam.transpose()))
        (pix, mask) = cloudToPixels(cam, pts_wrt_cam)

        #intensity = data[mask, 3]
        #colors = heatColorMapFast(intensity, 0, 100)
        #I[pix[1,mask], pix[0,mask], :] = colors[0,:,:]
        
        colors = 0*pts_wrt_cam.transpose() + 0
        print I[pix[1,mask], pix[0,mask], :].shape
        print colors[mask, :].shape
        # BGR -> RGB
        colors[ mask, 0] = I[pix[1,mask], pix[0,mask], 2] 
        colors[ mask, 1] = I[pix[1,mask], pix[0,mask], 1] 
        colors[ mask, 2] = I[pix[1,mask], pix[0,mask], 0] 

        #cv2.imshow('vid', I)
        #cv2.waitKey(5)

        # transform data into IMU frame
        pts = data[:,0:3].transpose()
        pts = np.vstack((pts,np.ones((1,pts.shape[1]))))
        pts = np.dot(T_from_l_to_i, pts)
        pts = np.dot(IMUTransforms[fnum,:,:], pts);
        pts = pts.transpose()

        # for exporting purposes
        pts_copy = array(pts[:,0:3])
        pts_copy = np.column_stack((pts_copy, array(data[:,3])))
        pts_copy = np.column_stack((pts_copy, fnum*np.ones((pts.shape[0],1))))
        all_data.append(pts_copy)


        if color_mode == 'CAMERA':
            lidarCloud = VtkPointCloud(pts[mask,0:3], colors[mask,:])
            actors.append(lidarCloud.get_vtk_color_cloud())
        elif color_mode == 'INTENSITY': 
            lidarCloud = VtkPointCloud(pts[:,0:3], data[:,3])
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
        ry += 0.005
    elif key == 'k':
        ry -= 0.005
    elif key == 'l':
        rx += 0.05
    elif key == 'j':
        rx -= 0.05
    elif key == 'o':
        rz += 0.005
    elif key == 'u':
        rz -= 0.005
    elif key == 'c':
        color_mode = 'CAMERA'
    elif key == 'v':
        color_mode = 'INTENSITY'
    elif key == 'x':
        print 'exporting data'
        export_data = np.row_stack(all_data)
        print export_data
        print export_data.shape
        np.savez_compressed('output_map2.npz', data=export_data)

    else:
        rerender = False
    R = euler_matrix(rx,ry,rz)[0:3,0:3]

    if rerender:
        for a in actors:
            cloud_r.RemoveActor(a)
        actors = [ ]
        clouds = [ ]
        all_data = [ ]
        #start_fn = start_fn + 5
        integrateClouds(ldr_map, imu_transforms, cloud_r, R, start_fn, num_fn, step)
        renderWindow.Render()
    print key
    print (rx, ry, rz)

if __name__ == '__main__': 
    video_filename = sys.argv[1] 
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    
    cam = getCameraParams()[cam_num - 1] 
    video_reader = VideoReader(video_filename)
    gps_reader = GPSReader(gps_filename)
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    ldr_map = loadLDRCamMap(sys.argv[2])

    # this has been flipped for the q50
    
    cloud_r.SetBackground(0., 0., 0.)
    cloud_r.SetViewport(0,0,1.0,1.0)
    integrateClouds(ldr_map, imu_transforms, cloud_r, R, start_fn, num_fn, step)

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

    renderWindowInteractor.AddObserver('KeyPressEvent', keypress)
    #renderWindowInteractor.AddObserver('TimerEvent', keypress)
    #timerId = renderWindowInteractor.CreateRepeatingTimer(1)
    renderWindowInteractor.Start()
