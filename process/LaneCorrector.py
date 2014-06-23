#!/usr/bin/python
# -*- coding: utf-8 -*-
#Usage: LaneCorrector.py folder/ cam.avi raw_data.npz multilane_points.npz

from ArgParser import parse_args
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from Q50_config import LoadParameters
from VtkRenderer import VtkPointCloud, VtkBoundingBox, VtkText, VtkImage
import numpy as np
from scipy.spatial import KDTree
import sys
from transformations import euler_from_matrix
import vtk
from VideoReader import *
from ColorMap import heatColorMapFast
from MapReproject import lidarPtsToPixels

def vtk_transform_from_np(np4x4):
    vtk_matrix = vtk.vtkMatrix4x4()
    for r in range(4):
        for c in range(4):
            vtk_matrix.SetElement(r, c, np4x4[r, c])
    transform = vtk.vtkTransform()
    transform.SetMatrix(vtk_matrix)
    return transform

def get_transforms(args):
    """ Gets the IMU transforms for a run """
    gps_reader = GPSReader(args['gps'])
    gps_data = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(gps_data)
    return imu_transforms

class LaneInteractorStyle (vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, iren, ren, parent):
        self.iren = iren
        self.ren = ren
        self.parent = parent
        self.picker = vtk.vtkPointPicker()
        self.picker.SetTolerance(0.01)
        self.iren.SetPicker(self.picker)

        self.moving = False
        self.InteractionProp = None
        self.color = None
        self.data_in = None

        self.num_nearby = 100
        self.SetMotionFactor(40.0)

        self.AutoAdjustCameraClippingRangeOff()
        self.ren.GetActiveCamera().SetClippingRange(0.01, 1600)

        self.AddObserver('LeftButtonPressEvent', self.LeftButtonPressEvent)
        self.AddObserver('LeftButtonReleaseEvent', self.LeftButtonReleaseEvent)
        self.AddObserver('RightButtonPressEvent', self.RightButtonPressEvent)
        self.AddObserver('RightButtonReleaseEvent', self.RightButtonReleaseEvent)
        self.AddObserver('MouseMoveEvent', self.MouseMoveEvent)

        self.AddObserver('MouseWheelForwardEvent', self.MouseWheelForwardEvent)
        self.AddObserver('MouseWheelBackwardEvent', self.MouseWheelBackwardEvent)

        # Add keypress event
        self.AddObserver('KeyPressEvent', self.KeyHandler)

    def MouseWheelForwardEvent(self, obj, event):
        print self.ren.GetActiveCamera().GetPosition()
        self.OnMouseWheelForward()

    def MouseWheelBackwardEvent(self, obj, event):
        print self.ren.GetActiveCamera().GetPosition()
        self.OnMouseWheelBackward()

    def LeftButtonPressEvent(self, obj, event):
        x, y = self.iren.GetEventPosition()
        self.picker.Pick(x, y, 0, self.ren)
        idx = self.picker.GetPointId()
        if idx >= 0 and self.getLane(self.picker.GetActor()) != None:
            self.InteractionProp = self.picker.GetActor()
            self.idx = idx
            self.moving = True

    def LeftButtonReleaseEvent(self, obj, event):
        lane = self.getLane(self.InteractionProp)
        if lane != None:
            print '(%d, %d)' % (lane, self.idx)
            for p in self.nextNearbyPoint(self.idx):
                self.color.SetTuple(p, (lane,))

            self.Render()
            
        self.moving = False
        self.idx = -1
        self.InteractionProp = None

    def RightButtonPressEvent(self, obj, event):
        self.OnLeftButtonDown()

    def RightButtonReleaseEvent(self, obj, event):
        self.OnLeftButtonUp()

    def MouseMoveEvent(self, obj, event):
        if self.moving:
            self.data_in = self.InteractionProp.GetMapper().GetInput()
            self.pos = self.data_in.GetPoints().GetData()
            self.color = self.data_in.GetPointData().GetScalars()

            old_pos = self.pos.GetTuple(self.idx)
            x, y = self.iren.GetEventPosition()

            disp_obj_center = [0.] * 3
            new_pick_point = [0.] * 4
            self.ComputeWorldToDisplay(self.ren, old_pos[0], old_pos[1],
                                       old_pos[2], disp_obj_center)
            self.ComputeDisplayToWorld(self.ren, x, y, disp_obj_center[2],
                                       new_pick_point)

            new_pos = new_pick_point[:-1]
            new_pos[2] = old_pos[2]

            new_pos = np.array(new_pos)
            old_pos = np.array(old_pos)

            change = new_pos - old_pos
            for p in self.nextNearbyPoint(self.idx):
                if p >= 0 and p < self.data_in.GetNumberOfElements(0):
                    p_old_pos = np.array(self.pos.GetTuple(p))
                    alpha = 1.0 - abs(self.idx - p) / float(self.num_nearby)
                    p_new_pos = p_old_pos + change * alpha
                    self.pos.SetTuple(p, tuple(p_new_pos))
                    self.color.SetTuple(p, (5,))

            self.Render()

    def KeyHandler(self, obj, event):
        key = self.iren.GetKeySym()
        if not self.moving:
            if key == 'Up':
                # Increase the number of points selected
                self.num_nearby += 1
                self.setModeText(self.getModeText('single'))
            elif key == 'Down':
                # Decrease the number of points selected
                num = self.num_nearby - 1
                self.num_nearby = num if num > 0 else 1
                self.setModeText(self.getModeText('single'))
            elif key in [str(i) for i in xrange(self.parent.num_lanes)]:
                if key == '3':
                    # Workaround: 3 toggles stereo rendering. Turn it on so later
                    # it is turned off
                    self.parent.win.StereoRenderOn()

                self.setModeText(self.getModeText(key))
            elif key == 'x':
                # Export Data
                self.parent.exportData('out.npz')
            elif key == '0':
                self.parent.count = 0
            elif key == 'r':
                self.parent.record = ~self.parent.record
                if self.parent.record:
                    self.parent.startVideo()
                else:
                    self.parent.closeVideo()
            else:
                pass

    def getModeText(self, mode):
        if mode == 'single':
            return 'Single Lane - ' + str(self.num_nearby)
        elif mode in [str(i) for i in xrange(self.parent.num_lanes)]:
            return 'Lane ' + mode + ' - All points'
        else:
            return 'All Lanes - ' + str(self.num_nearby)

    def setModeText(self, text):
        self.parent.selectModeActor.SetInput(text)
        self.parent.selectModeActor.Modified()

    def Render(self):
        self.data_in.Modified()
        self.iren.GetRenderWindow().Render()
    
    def nextNearbyPoint(self, mid):
        n = self.num_nearby
        i = mid - n + 1
        while i < mid + n:
            yield i
            i += 1

    def getLane(self, actor):
        if actor and actor in self.parent.interp_actor:
            return self.parent.interp_actor.index(actor)
        else:
            return None

class Blockworld:

    def __init__(self):
        args = parse_args(sys.argv[1], sys.argv[2])

        self.start = 0
        self.step = 10
        self.end = self.step * 500
        self.count = 0

        ##### Grab all the transforms ######
        self.imu_transforms = get_transforms(args)
        self.trans_wrt_imu = self.imu_transforms[
            self.start:self.end:self.step, 0:3, 3]
        self.params = args['params']
        self.lidar_params = self.params['lidar']
        self.T_from_i_to_l = np.linalg.inv(self.lidar_params['T_from_l_to_i'])
        cam_num = args['cam_num']
        self.cam_params = self.params['cam'][cam_num-1]

        # Whether to write video
        self.record = False

        ###### Set up the renderers ######
        self.cloud_ren = vtk.vtkRenderer()
        self.cloud_ren.SetViewport(0,0,1.0,1.0)
        self.cloud_ren.SetBackground(0, 0, 0)

        self.img_ren = vtk.vtkRenderer()
        self.img_ren.SetViewport(0.75,0.0,1.0,0.25)
        # self.img_ren.SetInteractive(False)
        self.img_ren.SetBackground(0.1, 0.1, 0.1)

        self.win = vtk.vtkRenderWindow()
        self.win.StereoCapableWindowOff()
        self.win.AddRenderer(self.cloud_ren)
        self.win.AddRenderer(self.img_ren)
        self.win.SetSize(800, 400)

        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren.SetRenderWindow(self.win)

        ###### Cloud Actors ######
        print 'Adding raw points'
        raw_npz = np.load(sys.argv[3])
        pts = raw_npz['data']
        raw_cloud = VtkPointCloud(pts[:, :3], pts[:, 3])
        raw_actor = raw_cloud.get_vtk_cloud(zMin=0, zMax=100)
        raw_actor.GetProperty().SetPointSize(5)
        raw_actor.SetPickable(0)
        self.cloud_ren.AddActor(raw_actor)

        print 'Loading interpolated lanes'
        npz = np.load(sys.argv[4])
        self.num_lanes = int(npz['num_lanes'])

        interp_lanes = [None] * self.num_lanes
        interp_times = [None] * self.num_lanes
        self.interp_cloud = [None] * self.num_lanes
        self.interp_actor = [None] * self.num_lanes

        for i in xrange(self.num_lanes):
            interp_lanes[i] = npz['lane' + str(i)]
            interp_times[i] = npz['time' + str(i)]
            # The intensity of the lane tells us which lane it is
            # Do not change this, it is very important
            num_pts = interp_lanes[i].shape[0]
            self.interp_cloud[i] = VtkPointCloud(interp_lanes[i][:, :3],
                                            np.ones((num_pts)) * i)
            self.interp_actor[i] = self.interp_cloud[i].get_vtk_cloud(zMin=0,
                                                                      zMax=self.num_lanes+1)
            self.interp_actor[i].GetProperty().SetPointSize(2)
            self.cloud_ren.AddActor(self.interp_actor[i])

        self.laneKDTree = KDTree(self.interp_cloud[0].xyz)

        # Use our custom mouse interactor
        self.mouseInteractor = LaneInteractorStyle(self.iren, self.cloud_ren, self)
        self.iren.SetInteractorStyle(self.mouseInteractor)

        # Tell the user which mode we are in
        selectMode = VtkText(self.mouseInteractor.getModeText('single'), (10, 10))
        self.selectModeActor = selectMode.get_vtk_text()
        self.cloud_ren.AddActor(self.selectModeActor)

        ###### 2D Projection Actors ######
        self.video_reader = VideoReader(args['video'])
        self.img_actor = None

        ###### Add Callbacks ######
        print 'Rendering'

        self.iren.Initialize()

        # Set up time
        self.iren.AddObserver('TimerEvent', self.update)
        self.timer = self.iren.CreateRepeatingTimer(100)


        self.iren.Start()

        
    def getCameraPosition(self, t):
        position = self.imu_transforms[t - self.step, 0:3, 3] +\
                   np.array([0, 0, 75.0])
        focal_point = self.imu_transforms[t, 0:3, 3]
        return position, focal_point


    def exportData(self, file_name):
        lanes = {}
        lanes['num_lanes'] = self.num_lanes
        for num in xrange(self.num_lanes):
            lane = self.interp_cloud[num].xyz[:, :3]
            offset = np.vstack((lane[1:, :], np.zeros((1,3))))
            lane = np.hstack((lane, offset)) 
            lanes['lane' + str(num)] = lane
        
        np.savez(file_name, **lanes)
            
    def update(self, iren, event):
        # Transform the car
        # t = self.start + self.step * self.count
        t = self.start + self.step
        self.img_ren.RemoveActor(self.img_actor)
        
        self.video_reader.setFrame(t - self.step)
        (success, self.I) = self.video_reader.getNextFrame()

        self.I = self.projectPointsOnImg(self.I, t)
        vtkimg = VtkImage(self.I)
        self.img_actor = vtkimg.get_vtk_image()
        self.img_ren.AddActor(self.img_actor)

        if self.count == 0:
            imu_transform = self.imu_transforms[t, :,:]

            # Set camera position
            cloud_cam = self.cloud_ren.GetActiveCamera()
            position, focal_point = self.getCameraPosition(t)

            cloud_cam.SetPosition(position)
            cloud_cam.SetFocalPoint(focal_point)
            cloud_cam.SetViewUp(0, 0, 1)

            self.img_ren.ResetCamera()
            self.img_ren.GetActiveCamera().SetClippingRange(100, 100000)

        
        if self.record:
            self.writeVideo()

        iren.GetRenderWindow().Render()

        self.count += 1

    def projectPointsOnImg(self, I, t):
        car_pos = self.imu_transforms[t, 0:3, 3]
        (d, i) = self.laneKDTree.query(car_pos)
        
        for num in xrange(self.num_lanes):
            lane = self.interp_cloud[num].xyz[i:i+100, :3]
            pix, mask = lidarPtsToPixels(lane, self.imu_transforms[t,:,:],
                                         self.T_from_i_to_l, self.cam_params)
        
            intensity = np.ones((pix.shape[0], 1)) * num
            heat_colors = heatColorMapFast(intensity, 0, 5)
            for p in range(4):
                I[pix[1, mask]+p, pix[0, mask], :] = heat_colors[0,:,:]
                I[pix[1, mask], pix[0, mask]+p, :] = heat_colors[0,:,:]

        return I

    # Video Recording
    def startVideo(self):
        self.win2img = vtk.vtkWindowToImageFilter()
        self.win2img.SetInput(self.win)
        self.videoWriter = vtk.vtkFFMPEGWriter()
        self.videoWriter.SetFileName('multilane.avi')
        self.videoWriter.SetInputConnection(self.win2img.GetOutputPort())
        self.videoWriter.SetRate(10)  # 10 fps
        self.videoWriter.SetQuality(2)  # Highest
        self.videoWriter.SetBitRate(1000)  # kilobits/s
        self.videoWriter.SetBitRateTolerance(1000)
        self.videoWriter.Start()

    def writeVideo(self):
        self.win2img.Modified()
        self.videoWriter.Write()

    def closeVideo(self):
        self.videoWriter.End()
        self.videoWriter.Delete()
        self.win2img.Delete()

if __name__ == '__main__':
    blockworld = Blockworld()
