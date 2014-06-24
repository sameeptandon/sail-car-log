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
import math
from collections import deque

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

def load_ply(ply_file):
    """ Loads a ply file and returns an actor """
    reader = vtk.vtkPLYReader()
    reader.SetFileName(ply_file)
    reader.Update()

    ply_mapper = vtk.vtkPolyDataMapper()
    ply_mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(ply_mapper)
    return actor

class Undoer:

    class Change:
        def __init__(self, lane, idx, num_to_move, change):
            self.lane = lane
            self.idx = idx
            self.num_to_move = num_to_move
            self.change = change

        def __str__(self):
            return '(%d, %d) %s' % (self.lane, self.idx, self.change)

    def __init__(self, lane_actors, parent):
        self.lane_actors = lane_actors
        self.undo_queue = deque(maxlen=100)
        self.redo_queue = deque(maxlen=100)
        self.parent = parent

    def __str__(self):
        s = 'undo:\n'
        for i in self.undo_queue:
            s += '\t' + str(i) + '\n'
        s+= 'redo:\n'
        for i in self.redo_queue:
            s +=  '\t' + str(i) + '\n'
        return s

    def undo(self):
        try:
            undo = self.undo_queue.pop()
            self.redo_queue.append(undo)
            self.performChange(undo)
        except IndexError:
            pass

    def redo(self):
        try:
            redo = self.redo_queue.pop()
            self.undo_queue.append(redo)
            self.performChange(redo, -1)
        except IndexError:
            pass

    def addChange(self, lane, idx, num_to_move, change):
        change = Undoer.Change(lane, idx, num_to_move, change)
        try:
            # Try to append
            self.undo_queue.append(change)
        except IndexError:
            # Remove the oldest change
            self.undo_queue.popleftn()
            # Add the new change
            self.undo_queue.append(change)
        self.redo_queue.clear()

    def performChange(self, change, direction=1):
        # def movePoint (self, lane, point, num_to_move, change):
        self.parent.movePoint(change.lane, change.idx, change.num_to_move,
                              direction * np.array(change.change))

class LaneInteractorStyle (vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, iren, ren, parent):
        self.iren = iren
        self.ren = ren
        self.parent = parent
        self.picker = vtk.vtkPointPicker()
        self.picker.SetTolerance(0.01)
        self.iren.SetPicker(self.picker)

        self.undoer = Undoer(self.parent.interp_actors, self)

        self.moving = False
        self.InteractionProp = None
        self.color = None
        self.data_in = None

        self.mode = 'single'

        self.num_to_move = 100
        self.SetMotionFactor(40.0)

        self.AutoAdjustCameraClippingRangeOff()
        self.ren.GetActiveCamera().SetClippingRange(0.01, 500)

        self.AddObserver('MouseWheelForwardEvent', self.MouseWheelForwardEvent)
        self.AddObserver('MouseWheelBackwardEvent', self.MouseWheelBackwardEvent)

        self.AddObserver('RightButtonPressEvent', self.RightButtonPressEvent)
        self.AddObserver('RightButtonReleaseEvent', self.RightButtonReleaseEvent)
        self.AddObserver('LeftButtonPressEvent', self.LeftButtonPressEvent)
        self.AddObserver('LeftButtonReleaseEvent', self.LeftButtonReleaseEvent)
        self.AddObserver('MouseMoveEvent', self.MouseMoveEvent)

        # Add keypress event
        self.AddObserver('KeyPressEvent', self.KeyHandler)

    def MouseWheelForwardEvent(self, obj, event):
        self.OnMouseWheelForward()

    def MouseWheelBackwardEvent(self, obj, event):
        self.OnMouseWheelBackward()

    def RightButtonPressEvent(self, obj, event):
        x, y = self.iren.GetEventPosition()
        self.FindPokedRenderer(x, y)

        # Only allow rotation for the cloud camera
        if self.GetCurrentRenderer() == self.ren:
            self.OnLeftButtonDown()

    def RightButtonReleaseEvent(self, obj, event):
        self.OnLeftButtonUp()

    def LeftButtonPressEvent(self, obj, event):
        x, y = self.iren.GetEventPosition()
        self.picker.Pick(x, y, 0, self.ren)
        idx = self.picker.GetPointId()
        if idx >= 0 and self.getLane(self.picker.GetActor()) != None:
            self.InteractionProp = self.picker.GetActor()
            self.idx = idx
            self.moving = True
            self.startPos = self.getPointPosition(self.InteractionProp, self.idx)

    def LeftButtonReleaseEvent(self, obj, event):
        if self.moving:
            lane = self.getLane(self.InteractionProp)
            if lane != None:
                print '(%d, %d)' % (lane, self.idx)
                for p in self.nextNearbyPoint(self.idx):
                    self.color.SetTuple(p, (lane,))

                self.Render()

                endPos = self.getPointPosition(self.InteractionProp, self.idx)
                change = self.startPos - endPos
                self.undoer.addChange(lane, self.idx, self.num_to_move, change)

            self.moving = False
            self.idx = -1
            self.InteractionProp = None
            self.startPos = None

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

            lane = self.getLane(self.InteractionProp)
            # Move the point
            self.movePoint(lane, self.idx, self.num_to_move, change)
            
            # Color the points that we are moving
            for p in self.nextNearbyPoint(self.idx):
                if p >= 0 and p < self.data_in.GetNumberOfElements(0):
                    self.color.SetTuple(p, (5,))

            self.Render()

    def movePoint (self, lane, idx, num_to_move, change):
        lane_actor = self.parent.interp_actors[lane]
        data_in = lane_actor.GetMapper().GetInput()
        pos = data_in.GetPoints().GetData()
        for p in self.nextNearbyPoint(idx):
            if p >= 0 and p < data_in.GetNumberOfElements(0):
                p_old_pos = np.array(pos.GetTuple(p))
                alpha = abs(idx - p) / float(num_to_move)
                alpha = (math.cos(alpha * math.pi) + 1) / 2.
                p_new_pos = p_old_pos + change * alpha
                pos.SetTuple(p, tuple(p_new_pos))

        data_in.Modified()
        
    def KeyHandler(self, obj, event):
        key = self.iren.GetKeySym()
        if not self.moving:
            if key == 'Up':
                # Increase the number of points selected
                self.num_to_move += 1
                self.mode = 'single'
            elif key == 'Down':
                # Decrease the number of points selected
                num = self.num_to_move - 1
                self.num_to_move = num if num > 0 else 1
                self.mode = 'single'
            elif key in [str(i) for i in xrange(self.parent.num_lanes)]:
                if key == '3':
                    # Workaround: 3 toggles stereo rendering. Turn it on so later
                    # it is turned off
                    self.parent.win.StereoRenderOn()
                self.mode = key
            elif key == 'x':
                # Export Data
                self.parent.exportData('out.npz')
            elif key == 'space':
                self.parent.running = not self.parent.running
            elif key == 'z':
                self.undoer.undo()
                self.Render()
            elif key == 'y':
                self.undoer.redo()
                self.Render()
            elif key == 'r':
                self.parent.record = not self.parent.record
                if self.parent.record:
                    print 'Recording'
                    self.startVideo()
                else:
                    print 'Saving recording'
                    self.closeVideo()
            else:
                pass

    def updateModeText(self):
        frame_num = self.parent.count
        mode = self.mode

        txt = '(%d) ' % frame_num
        if mode == 'single':
            text = txt + 'Single Lane - %d' % self.num_to_move
        elif mode in [str(i) for i in xrange(self.parent.num_lanes)]:
            text = txt + 'Lane %s - All points' % mode
        else:
            text = txt + 'All Lanes - %d' % self.num_to_move

        self.parent.selectModeActor.SetInput(text)
        self.parent.selectModeActor.Modified()

    def Render(self):
        self.data_in.Modified()
        self.iren.GetRenderWindow().Render()
    
    def nextNearbyPoint(self, mid):
        n = self.num_to_move
        i = mid - n + 1
        while i < mid + n:
            yield i
            i += 1

    def getLane(self, actor):
        if actor and actor in self.parent.interp_actors:
            return self.parent.interp_actors.index(actor)
        else:
            return None

    def getPointPosition(self, actor, index):
        if self.getLane(actor) != None:
            pos = actor.GetMapper().GetInput().GetPoints().GetData()
            return np.array(pos.GetTuple(index))

    # Video Recording
    def startVideo(self):
        self.win2img = vtk.vtkWindowToImageFilter()
        self.win2img.SetInput(self.parent.win)

        self.videoWriter = vtk.vtkFFMPEGWriter()
        self.videoWriter.SetFileName('multilane.avi')
        self.videoWriter.SetInputConnection(self.win2img.GetOutputPort())
        self.videoWriter.SetRate(15) 
        self.videoWriter.SetQuality(2) # 2 is the highest
        self.videoWriter.SetBitRate(25000)
        self.videoWriter.SetBitRateTolerance(2000)
        self.videoWriter.Start()

    def writeVideo(self):
        self.win2img.Modified()
        self.videoWriter.Write()

    def closeVideo(self):
        self.videoWriter.End()

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
        self.running = True

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
        self.interp_cloud = [None] * self.num_lanes
        self.interp_actors = [None] * self.num_lanes

        for i in xrange(self.num_lanes):
            interp_lanes[i] = npz['lane' + str(i)]
            # The intensity of the lane tells us which lane it is
            # Do not change this, it is very important
            num_pts = interp_lanes[i].shape[0]
            self.interp_cloud[i] = VtkPointCloud(interp_lanes[i][:, :3],
                                            np.ones((num_pts)) * i)
            self.interp_actors[i] = self.interp_cloud[i].get_vtk_cloud(zMin=0,
                                                                      zMax=self.num_lanes+1)
            self.interp_actors[i].GetProperty().SetPointSize(2)
            self.cloud_ren.AddActor(self.interp_actors[i])

        self.laneKDTree = KDTree(self.interp_cloud[0].xyz)

        # self.calculateZError(pts)

        print 'Adding car'
        self.car = load_ply('../mapping/viz/gtr.ply')
        self.car.SetPickable(0)
        self.car.GetProperty().LightingOff()
        self.cloud_ren.AddActor(self.car)

        # Use our custom mouse interactor
        self.mouseInteractor = LaneInteractorStyle(self.iren, self.cloud_ren, self)
        self.iren.SetInteractorStyle(self.mouseInteractor)

        # Tell the user which mode we are in
        selectMode = VtkText('', (10, 10))
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

    def calculateZError(self, pts):
        # Calculates median z-error of interpolated lanes to points
        tree = KDTree(pts[:, :3])
        for num in xrange(self.num_lanes):
            lane = self.interp_cloud[num].xyz[:4910, :]
            z_dist = []
            for p in lane:
                (d, i) = tree.query(p)
                z_dist.append(abs(p[2] - pts[i, 2]))
            z_dist = np.array(z_dist)
            print 'Median Error in Lane %d: %f' % (num, np.median(z_dist)) 

    def getCameraPosition(self, t):
        offset = np.array([-75.0, 0, 25.0])
        position = np.dot(self.imu_transforms[t,0:3,0:3], offset)

        position += self.imu_transforms[t, 0:3, 3] + position
        focal_point = self.imu_transforms[t + 10 * self.step, 0:3, 3]
        return position, focal_point


    def exportData(self, file_name):
        lanes = {}
        lanes['num_lanes'] = self.num_lanes
        for num in xrange(self.num_lanes):
            lane = self.interp_cloud[num].xyz[:, :3]
            offset = np.vstack((lane[1:, :], np.zeros((1,3))))
            lane = np.hstack((lane, offset)) 
            lanes['lane' + str(num)] = lane

        print 'Saved', file_name
        np.savez(file_name, **lanes)
            
    def update(self, iren, event):
        # Transform the car
        t = self.start + self.step * self.count
        cloud_cam = self.cloud_ren.GetActiveCamera()
        # self.video_reader.setFrame(t - 1)
        # (success, I) = self.video_reader.getNextFrame()
        
        while self.video_reader.framenum <= t:
            (success, self.I) = self.video_reader.getNextFrame()

        I = self.I.copy()
        I = self.projectPointsOnImg(I, t)
        vtkimg = VtkImage(I)

        self.img_ren.RemoveActor(self.img_actor)
        self.img_actor = vtkimg.get_vtk_image()
        self.img_ren.AddActor(self.img_actor)

        if self.running:
            # Set camera position
            position, focal_point = self.getCameraPosition(t)

            cloud_cam.SetPosition(position)
            cloud_cam.SetFocalPoint(focal_point)

            imu_transform = self.imu_transforms[t, :, :]
            transform = vtk_transform_from_np(imu_transform)
            transform.RotateZ(90)
            transform.Translate(-2, -3, -2)
            self.car.SetUserTransform(transform)

        if self.count == 0:
            cloud_cam.SetViewUp(0, 0, 1)
            self.img_ren.ResetCamera()
            self.img_ren.GetActiveCamera().SetClippingRange(100, 100000)
        
        self.mouseInteractor.updateModeText()

        iren.GetRenderWindow().Render()

        if self.record:
            self.mouseInteractor.writeVideo()
        
        if self.running or self.count == 0:
            self.count += 1

    def projectPointsOnImg(self, I, t):
        car_pos = self.imu_transforms[t, 0:3, 3]

        (d, closest_idx) = self.laneKDTree.query(car_pos)
        nearby_idx = np.array(self.laneKDTree.query_ball_point(car_pos, r=100.0))
        nearby_idx = nearby_idx[nearby_idx > closest_idx]

        for num in xrange(self.num_lanes):
            lane = self.interp_cloud[num].xyz[nearby_idx, :3]

            pix, mask = lidarPtsToPixels(lane, self.imu_transforms[t,:,:],
                                         self.T_from_i_to_l, self.cam_params)
            intensity = np.ones((pix.shape[0], 1)) * num
            heat_colors = heatColorMapFast(intensity, 0, 5)
            for p in range(4):
                I[pix[1, mask]+p, pix[0, mask], :] = heat_colors[0,:,:]
                I[pix[1, mask], pix[0, mask]+p, :] = heat_colors[0,:,:]
                I[pix[1, mask]-p, pix[0, mask], :] = heat_colors[0,:,:]
                I[pix[1, mask], pix[0, mask]-p, :] = heat_colors[0,:,:]

        return I


if __name__ == '__main__':
    blockworld = Blockworld()
