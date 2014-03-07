from Q50_config import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from LidarTransforms import *
from VtkRenderer import *
from transformations import euler_matrix
import numpy as np
import vtk

global actors
global clouds 
global cloud_r
global renderWindow
global rx
global ry
global rz
global R

actors =  []
clouds = [ ]
cloud_r = vtk.vtkRenderer()
renderWindow = vtk.vtkRenderWindow()
(rx,ry,rz) = (0,-0.01,-0.04)
#(rx, ry, rz) =  (0.004999999999999988, -0.035000000000000002, -0.10499999999999998)
#(rx,ry,rz) = (0.009999999999999988, -0.030000000000000002, -0.04499999999999996)
#(rx,ry,rz) = (0.014999999999999989, 0.005000000000000002, -0.03499999999999996)
#(rx,ry,rz) = (0.014999999999999989, 0.005000000000000002, -0.13499999999999995)
#(rx,ry,rz) = (0.0500000000000016, -0.015, 0.035)
R = euler_matrix(rx,ry,rz)[0:3,0:3]


def integrateClouds(ldr_map, GPSData, renderer, R):
    step = 2
    num_steps = 200 
    #offset = 3500
    offset = 0
   
    roll_start = -deg2rad(GPSData[offset,8])
    pitch_start = deg2rad(GPSData[offset,7])
    yaw_start = -deg2rad(GPSData[offset,9])

    print roll_start * 180/np.pi
    print pitch_start * 180/np.pi
    print yaw_start * 180/np.pi
    base_R_to_i_from_w = R_to_i_from_w(roll_start, pitch_start, yaw_start) 

    dx = WGS84toENU(GPSData[offset,1:4], GPSData[offset:offset+step*num_steps+10,1:4])
    dx = np.dot(base_R_to_i_from_w, dx)
    dx = dx.astype(np.float32)
    rot_dx = np.dot(R, dx);
    rot_dx = rot_dx.transpose()
    gpsPointCloud = VtkPointCloud(rot_dx[:,0:3], rot_dx[:,0]-rot_dx[:,0]+10)
    clouds.append(gpsPointCloud)
    actors.append(gpsPointCloud.get_vtk_cloud())
    renderer.AddActor(actors[-1])
    for t in range(num_steps):
        data = loadLDR(ldr_map[offset+t*step])
        # filter out the roof rack
        dist = np.sqrt(np.sum( data[:, 0:3] ** 2, axis = 1))
        data = data[ dist > 3, :]
        #data = data[ data[:,3] > 60 ] 
        #data = data[ data[:,0] > 0 ]
        #data = data[ np.abs(data[:,1]) < 5]
        #data = data[ data[:,2] < -1.5]
        data[:,0:3] += rot_dx[t*step,:]
        lidarCloud = VtkPointCloud(data[:,0:3], data[:,3])
        clouds.append(lidarCloud)
        actors.append(lidarCloud.get_vtk_cloud(zMin=0, zMax=255))
        renderer.AddActor(actors[-1])


def get_lidar_vtk_actor(data):
    return lidarCloud.get_vtk_cloud(zMin=0, zMax=255)

def keypress(obj, event):
    global actors
    global clouds 
    global cloud_r
    global renderWindow
    global rx
    global ry
    global rz
    global R
    key = obj.GetKeySym()
    rerender = True
    if key == 'i':
        ry += 0.05
    elif key == 'k':
        ry -= 0.05
    elif key == 'l':
        rx += 0.05
    elif key == 'j':
        rx -= 0.05
    elif key == 'o':
        rz += 0.005
    elif key == 'u':
        rz -= 0.005
    else:
        rerender = False
    R = euler_matrix(rx,ry,rz)[0:3,0:3]

    if rerender: 
        for a in actors:
            cloud_r.RemoveActor(a)
        actors = [ ]
        clouds = [ ]
        integrateClouds(ldr_map, GPSData, cloud_r, R)
        renderWindow.Render()
    print key
    print (rx, ry, rz)

if __name__ == '__main__': 
    video_filename = sys.argv[1] 
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    gps_reader = GPSReader(gps_filename)
    GPSData = gps_reader.getNumericData() 
    ldr_map = loadLDRCamMap(sys.argv[2])

    # this has been flipped for the q50
    
    cloud_r.SetBackground(0., 0., 0.)
    cloud_r.SetViewport(0,0,1.0,1.0)
    integrateClouds(ldr_map, GPSData, cloud_r, R)

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
    renderWindowInteractor.Start()
