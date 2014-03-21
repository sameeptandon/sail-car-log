from Q50_config import *
from ArgParser import *
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
import cv2
import SocketServer
import threading


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
global frame_nums
global orig_images
global ctx
global cty
global ctz
global crx
global cry
global crz
global cT
global cR
global paramInit

def ParametersToString():
    global rx
    global ry
    global rz
    global crx
    global cry
    global crz

    return "%f,%f,%f,%f,%f,%f\n" % (rx,ry,rz,crx,cry,crz)

class RequestHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        global rx
        global ry
        global rz
        global crx
        global cry
        global crz
        global R
        global cR
        global paramInit 

        data = self.request[0].strip()
        print data
        (rx,ry,rz,crx,cry,crz) = map(lambda x: float(x), data.split(','))
        
        R = euler_matrix(rx,ry,rz)[0:3,0:3].transpose()
        cR = euler_matrix(crx, cry, crz)[0:3,0:3]
        if paramInit:
            keypressUpdate('r')
        paramInit = True


class ThreadedServer(threading.Thread):
    def __init__(self):
        self.server = None
        threading.Thread.__init__(self)
    def run(self):
        if self.server == None:
            address = ('localhost', int(sys.argv[4]))
            self.server = SocketServer.UDPServer(address, RequestHandler)
        print 'starting server'
        self.server.serve_forever()


actors =  []
clouds = [ ]
all_data = [ ] 
cloud_r = vtk.vtkRenderer()
renderWindow = vtk.vtkRenderWindow()

paramInit = False

#(rx,ry,rz) = (-0.02, -0.013900000000000006, -0.039999999999999994)
#(rx,ry,rz) = (-0.11000000000000003, -0.0074000000000002, -0.029)
#(rx,ry,rz) = (-0.08, -0.0084000000000002, -0.031)
#(rx,ry,rz) = (-0.05499999999999999, -0.0084000000000002, -0.03)
#(rx,ry,rz) = (-0.05499999999999999, -0.0079000000000002, -0.038000000000000006)

(ctx, cty, ctz) = \
        (-0.07, 0.325, 0.16)
cT = np.array([ctx, cty, ctz])
#(ctx, cty, ctz, crx, cry, crz) = \
#        (-0.07, 0.325, 0.16, 0.0375, 0.008599999999999993, 0.01650000000000001)
#(crx,cry,crz) = (0.04650000000000001, 0.021100000000000004, 0.01650000000000001)
#(crx,cry,crz) = (0.045500000000000001, 0.019100000000000006, 0.00150000000000001)
#(crx,cry,crz) = (0.047000000000000005, 0.019600000000000006, 0.01150000000000001)
#(crx,cry,crz) = (0.04700000000000001, 0.02210000000000001, 0.01150000000000001)
#(crx,cry,crz) = (0.05200000000000001, 0.018100000000000005, 0.01150000000000001)
#(crx,cry,crz) = (0.04650000000000001, 0.018100000000000005, -0.00349999999999999)
#(crx,cry,crz) = (0.046000000000000006, 0.0221, 0.01150000000000001)
#(crx,cry,crz) = (0.046000000000000006, 0.0191, -0.013499999999999991)

#(crx,cry,crz) = (0.046000000000000006, 0.012099999999999993, 0.021500000000000012)
#cR = euler_matrix(crx, cry, crz)[0:3,0:3]
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

start_fn = 2050
num_fn = 25
step = 10

frame_nums =  [start_fn + 20]
#frame_nums =  range(10,150,20)
orig_images = [ ]
WINDOW = 5*50

color_mode = 'INTENSITY'


def localMapToPixels(map_data, imu_transforms_t, cam):
    global R
    global cT
    global cR
        
    # load nearby map frames
    pts_wrt_imu_0 = array(map_data[:,0:3]).transpose()
    pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0, 
        np.ones((1,pts_wrt_imu_0.shape[1]))))
    # transform points from imu_0 to imu_t
    pts_wrt_imu_t = np.dot( np.linalg.inv(imu_transforms_t), pts_wrt_imu_0)
    # transform points from imu_t to lidar_t
    T_from_i_to_l = np.eye(4)
    T_from_i_to_l[0:3,0:3] = R.transpose()
    pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t);
    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cT

    pts_wrt_camera_t = np.dot(cR, np.dot(R_to_c_from_l_old(0), 
            pts_wrt_camera_t.transpose()))
    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    print R
    print cT
    print cR
    return (pix, mask)

def showCloudOnImage(img, t, imu_transform_t, cam):
    global all_data
    map_data = np.row_stack(all_data)
    mask_window = (map_data[:,4] < t + WINDOW) & (map_data[:,4] > t );
    map_data_copy = array(map_data[mask_window, :]);

    # reproject
    (pix, mask) = localMapToPixels(map_data_copy, imu_transform_t, cam); 

    # draw 
    I = img.copy()
    intensity = map_data_copy[mask, 3]
    heat_colors = heatColorMapFast(intensity, 0, 100)
    for p in range(4):
        I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
        I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]
        I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
        I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]

    cv2.imshow('vid' + str(t), cv2.pyrDown(I))
    cv2.waitKey(1)

def showCloudOnImages(imgs, frame_nums, imu_transforms, cam):
    for t in range(len(imgs)):
        showCloudOnImage(imgs[t], frame_nums[t], imu_transforms[frame_nums[t],:,:], cam)


def loadImages(video_reader, framenums):
    imgs = [ ]
    for f in frame_nums:
        video_reader.setFrame(f-1)
        (success, I) = video_reader.getNextFrame()
        imgs.append(I.copy())
    return imgs


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

def integrateClouds(ldr_map, IMUTransforms, renderer, offset, num_steps, step):
    global R
    start = offset
    end = offset + num_steps*step

    #video_reader1.setFrame(start)
    #video_reader2.setFrame(start)

    trans_wrt_imu = IMUTransforms[start:end,0:3,3]
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
        data = data[ data[:,3] > 60 ] 
        #data = data[ data[:,0] > 0 ]
        #data = data[ np.abs(data[:,1]) < 2.2]
        #data = data[ data[:,2] < -1.5]
        #data = data[ data[:,2] > -2.5]

        # transform data into IMU frame
        pts = data[:,0:3].transpose()
        pts = np.vstack((pts,np.ones((1,pts.shape[1]))))
        T_from_l_to_i[0:3,0:3] = R
        pts = np.dot(T_from_l_to_i, pts)
        pts = np.dot(IMUTransforms[fnum,:,:], pts);
        pts = pts.transpose()

        # for exporting purposes
        pts_copy = array(pts[:,0:3])
        pts_copy = np.column_stack((pts_copy, array(data[:,3])))
        pts_copy = np.column_stack((pts_copy, fnum*np.ones((pts.shape[0],1))))
        all_data.append(pts_copy)


        if color_mode == 'CAMERA':
            lidarCloud = VtkPointCloud(pts[ (mask1 | mask2) ,0:3], colors[ (mask1 | mask2),:])
            actors.append(lidarCloud.get_vtk_color_cloud())
        elif color_mode == 'INTENSITY': 
            lidarCloud = VtkPointCloud(pts[:,0:3], data[:,3])
            actors.append(lidarCloud.get_vtk_cloud(zMin=0, zMax=255))
        clouds.append(lidarCloud)
        renderer.AddActor(actors[-1])


def keypress(obj, event):
    key = obj.GetKeySym()
    keypressUpdate(key)

def donothing(obj, event):
    time.sleep(0.1)



def keypressUpdate(key):
    global actors
    global clouds 
    global cloud_r
    global all_data
    global renderWindow
    global rx
    global ry
    global rz
    global crx
    global cry
    global crz
    global cR
    global R
    global start_fn
    global color_mode
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
    elif key == 'a':
        cry += 0.0005
    elif key == 'd':
        cry -= 0.0005
    elif key == 'w':
        crx += 0.0005
    elif key == 's':
        crx -= 0.0005
    elif key == 'plus':
        crz += 0.005
    elif key == 'underscore':
        crz -= 0.005
    elif key == 'c':
        color_mode = 'CAMERA'
    elif key == 'v':
        color_mode = 'INTENSITY'
    elif key == 'x':
        print 'exporting data'
        export_data = np.row_stack(all_data)
        print export_data
        print export_data.shape
        np.savez_compressed(sys.argv[3], data=export_data)
    elif key == 'r':
        print 'rerendering'
    else:
        rerender = False

    R = euler_matrix(rx,ry,rz)[0:3,0:3].transpose()
    cR = euler_matrix(crx,cry,crz)[0:3,0:3]
    if rerender:
        for a in actors:
            cloud_r.RemoveActor(a)
        actors = [ ]
        clouds = [ ]
        all_data = [ ]
        #start_fn = start_fn + 5
        integrateClouds(ldr_map, imu_transforms, cloud_r, start_fn, num_fn, step)
        showCloudOnImages(orig_images, frame_nums, imu_transforms, cam2)
        if key != 'r':
            sock.sendto('PARAMETER_UPDATE:'+sys.argv[4]+':'+ParametersToString(), ('localhost', 2929))
        renderWindow.Render()
    print key
    print 'R', (rx,ry,rz)
    print 'cR', (crx,cry,crz)

if __name__ == '__main__': 
    vfname = sys.argv[2]
    vidname = vfname.split('.')[0]
    vidname2 = vidname[:-1] + '2'
    video_filename2 = sys.argv[1] + '/' + vidname2 + '.avi'
    
    args = parse_args(sys.argv[1], sys.argv[2])

    gps_reader = GPSReader(args['gps'])
    cam1 = GetQ50CameraParams()[0] 
    cam2 = GetQ50CameraParams()[1] 
    video_reader1 = VideoReader(args['video'])
    video_reader2 = VideoReader(video_filename2)
    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    ldr_map = loadLDRCamMap(args['map'])

    orig_images = loadImages(video_reader2, frame_nums)

    # parameter server
    thr = ThreadedServer()
    thr.setDaemon(True)
    thr.start()
    import time
    time.sleep(1)
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto('PARAMETER_REQUEST:'+sys.argv[4]+'\n', ('localhost', 2929))
    while not paramInit:
        time.sleep(1)


    cloud_r.SetBackground(0., 0., 0.)
    cloud_r.SetViewport(0,0,1.0,1.0)
    integrateClouds(ldr_map, imu_transforms, cloud_r, start_fn, num_fn, step)
    showCloudOnImages(orig_images, frame_nums, imu_transforms, cam2)


    # Render Window
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(cloud_r)
    renderWindow.SetSize(1200, 600)

    axisActor = vtk.vtkAxisActor()
    axisActor.SetGridlineXLength(5)
    axisActor.SetGridlineYLength(5)
    axisActor.SetGridlineZLength(5)
    axisActor.DrawGridpolysOn()
    axisActor.DrawInnerGridlinesOn()
    cloud_r.AddActor(axisActor)

    # Interactor
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    renderWindowInteractor.SetInteractorStyle(mouseInteractor)
    renderWindow.Render()

    renderWindowInteractor.AddObserver('KeyPressEvent', keypress)
    renderWindowInteractor.AddObserver('TimerEvent', donothing)
    timerId = renderWindowInteractor.CreateRepeatingTimer(100)
    renderWindowInteractor.Start()
