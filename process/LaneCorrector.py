#!/usr/bin/python
# -*- coding: utf-8 -*-
#Usage: MultiLane.py folder/ video.avi data.npz interpolated_lanes.pickle

from ArgParser import parse_args
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from MultiLaneGenerator import MultiLane
from Q50_config import LoadParameters
from VtkRenderer import VtkPointCloud, VtkBoundingBox, VtkText
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.spatial import distance, KDTree
from sklearn import cluster
import sys
from transformations import euler_from_matrix
import vtk

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

def saveClusters(lanes, times, lane_idx, num_lanes):
    out = {}
    out['num_lanes'] = np.array(num_lanes)
    for i in xrange(num_lanes):
        mask = lanes[:, lane_idx] == i
        lane = lanes[mask]
        time = times[mask]

        lane = lane[:, :3]

        shifted = np.vstack((lane[1:, :], np.zeros((1, 3))))
        lane = np.hstack((lane, shifted))
        out['lane' + str(i)] = lane
        out['time' + str(i)] = time

    np.savez('multilane_points', **out)

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

        self.num_nearby = 10

        self.AddObserver('LeftButtonPressEvent', self.LeftButtonPressEvent)
        self.AddObserver('LeftButtonReleaseEvent', self.LeftButtonReleaseEvent)
        self.AddObserver('RightButtonPressEvent', self.RightButtonPressEvent)
        self.AddObserver('RightButtonReleaseEvent', self.RightButtonReleaseEvent)
        self.AddObserver('MouseMoveEvent', self.MouseMoveEvent)

        self.AddObserver('MouseWheelForwardEvent', self.MouseWheelForwardEvent)
        self.AddObserver('MouseWheelBackwardEvent', self.MouseWheelBackwardEvent)

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

        # Whether to write video
        self.record = False

        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(0, 0, 0)

        self.win = vtk.vtkRenderWindow()
        self.win.AddRenderer(self.ren)
        self.win.SetSize(800, 400)

        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren.SetRenderWindow(self.win)

        # Transforms
        self.imu_transforms = get_transforms(args)
        self.trans_wrt_imu = self.imu_transforms[
            self.start:self.end:self.step, 0:3, 3]
        self.params = args['params']
        self.lidar_params = self.params['lidar']

        print 'Adding raw points'
        raw_npz = np.load(sys.argv[3])
        pts = raw_npz['data']
        raw_cloud = VtkPointCloud(pts[:, :3], pts[:, 3])
        raw_actor = raw_cloud.get_vtk_cloud(zMin=0, zMax=100)
        raw_actor.GetProperty().SetPointSize(5)
        raw_actor.SetPickable(0)
        self.ren.AddActor(raw_actor)

        print 'Loading interpolated lanes'
        npz = np.load(sys.argv[4])
        num_lanes = int(npz['num_lanes'])

        interp_lanes = [None] * num_lanes
        interp_times = [None] * num_lanes
        interp_cloud = [None] * num_lanes
        self.interp_actor = [None] * num_lanes

        for i in xrange(num_lanes):
            interp_lanes[i] = npz['lane' + str(i)]
            interp_times[i] = npz['time' + str(i)]
            interp_cloud[i] = VtkPointCloud(interp_lanes[i][:, :3],
                                            np.ones((interp_lanes[i].shape[0]))
                                            * i)
            self.interp_actor[i] = interp_cloud[i].get_vtk_cloud(zMin=0, zMax=num_lanes+1)
            self.interp_actor[i].GetProperty().SetPointSize(2)
            self.ren.AddActor(self.interp_actor[i])


        self.mouseInteractor = LaneInteractorStyle(self.iren, self.ren, self)
        self.iren.SetInteractorStyle(self.mouseInteractor)

        selectMode = VtkText(self.getModetext('single'), (10, 10))
        self.selectModeActor = selectMode.get_vtk_text()
        self.ren.AddActor(self.selectModeActor)

        print 'Rendering'

        self.iren.Initialize()

        # Set up time
        self.iren.AddObserver('TimerEvent', self.update)
        self.timer = self.iren.CreateRepeatingTimer(100)

        # Add keypress event
        self.iren.AddObserver('KeyPressEvent', self.keyhandler)

        self.iren.Start()

    def getModetext(self, mode):
        if mode == 'single':
            return 'Single Lane - ' + str(self.mouseInteractor.num_nearby)
        else:
            return 'All Lanes - ' + str(self.mouseInteractor.num_nearby)

    def getCameraPosition(self, t):
        position = self.imu_transforms[t - self.step, 0:3, 3] +\
                   np.array([0, 0, 75.0])
        focal_point = self.imu_transforms[t, 0:3, 3]
        return position, focal_point

    def keyhandler(self, obj, event):
        key = obj.GetKeySym()
        
        if key == 'Up':
            self.mouseInteractor.num_nearby += 1
            self.selectModeActor.SetInput(self.getModetext('single'))
            self.selectModeActor.Modified()
        elif key == 'Down':
            num = self.mouseInteractor.num_nearby - 1
            self.mouseInteractor.num_nearby = num if num > 0 else 1
            self.selectModeActor.SetInput(self.getModetext('single'))
            self.selectModeActor.Modified()
            
        if key == 'a':
            self.mode = 'above'
        elif key == '0':
            self.count = 0
        elif key == 'r':
            self.record = ~self.record
            print self.record
            if self.record:
                self.startVideo()
            else:
                self.closeVideo()
        else:
            pass

    def update(self, iren, event):
        # Transform the car
        # t = self.start + self.step * self.count
        if self.count == 1:
            t = self.start + self.step

            imu_transform = self.imu_transforms[t, :,:]

            # Set camera position
            fren = iren.GetRenderWindow().GetRenderers().GetFirstRenderer()
            cam = fren.GetActiveCamera()
            position, focal_point = self.getCameraPosition(t)

            cam.SetPosition(position)
            cam.SetFocalPoint(focal_point)
            cam.SetViewUp(0, 0, 1)
            fren.ResetCameraClippingRange()
            cam.SetClippingRange(0.1, 1600)

            if self.record:
                self.writeVideo()
        
        iren.GetRenderWindow().Render()

        self.count += 1

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
