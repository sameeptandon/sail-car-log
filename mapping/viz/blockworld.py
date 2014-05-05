import numpy as np
from numpy.linalg import inv
import vtk
#import vtk.util.numpy_support as converter
from vtk.io import vtkPLYReader
import h5py
from pipeline_config import COLOR_OCTOMAP_H5_FILE,\
        GPS_FILE, EXPORT_START, EXPORT_STEP, EXPORT_NUM,\
        OCTOMAP_H5_FILE, STATIC_VTK_FILE, DYNAMIC_VTK_FILE, MAP_FILE
from VtkRenderer import VtkPointCloud, VtkBoundingBox
from transformations import euler_from_matrix
from RadarTransforms import loadRDRCamMap, loadRDR, calibrateRadarPts

from GPSTransforms import IMUTransforms
from GPSReader import GPSReader
from Q50_config import LoadParameters


def load_ply(ply_file):
    reader = vtkPLYReader()
    reader.SetFileName(ply_file)
    reader.Update()
    reader.Update()

    ply_mapper = vtk.vtkPolyDataMapper()
    ply_mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(ply_mapper)
    return actor


def vtk_transform_from_np(np4x4):
    vtk_matrix = vtk.vtkMatrix4x4()
    for r in range(4):
        for c in range(4):
            vtk_matrix.SetElement(r, c, np4x4[r, c])
    transform = vtk.vtkTransform()
    transform.SetMatrix(vtk_matrix)
    return transform


def load_octomap(octomap_h5_file, conf=0.9, wireframe=False):
    h5f = h5py.File(octomap_h5_file, 'r')
    octree_data = h5f['octree'][...]

    octree_data = octree_data[octree_data[:, 4] > conf]
    pts = vtk.vtkPoints()
    #vtk_pt_data = converter.numpy_to_vtk(np.ascontiguousarray(octree_data[:, 0:3]))
    #pts.SetData(vtk_pt_data)

    use_colors = octree_data.shape[1] > 5
    colors = vtk.vtkUnsignedCharArray()
    colors.SetNumberOfComponents(3)
    #color_data = np.ascontiguousarray(octree_data[:, 5:8])
    #colors = converter.numpy_to_vtk(color_data)
    #colors.SetName('ColorArray')
    #polydata.GetPointData().SetActiveScalars('ColorArray')

    for k in range(octree_data.shape[0]):
        pts.InsertNextPoint(*octree_data[k, 0:3])
        if use_colors:
            r = int(octree_data[k, 5])
            g = int(octree_data[k, 6])
            b = int(octree_data[k, 7])
            colors.InsertNextTupleValue((r, g, b))

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(pts)
    if use_colors:
        polydata.GetPointData().SetScalars(colors)

    cube = vtk.vtkCubeSource()
    cube.SetXLength(octree_data[0, 3])
    cube.SetYLength(octree_data[0, 3])
    cube.SetZLength(octree_data[0, 3])

    glyph = vtk.vtkGlyph3D()
    if use_colors:
        glyph.SetColorModeToColorByScalar()
    glyph.SetSourceConnection(cube.GetOutputPort())
    glyph.SetInput(polydata)
    glyph.ScalingOff()
    glyph.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(glyph.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    if wireframe:
        actor.GetProperty().SetRepresentationToWireframe()
        actor.GetProperty().SetLineWidth(1)
        actor.GetProperty().SetOpacity(0.2)
    actor.GetProperty().LightingOff()

    return actor


def load_vtk_cloud(vtk_cloud_file):
    reader = vtk.vtkDataSetReader()
    reader.SetFileName(vtk_cloud_file)
    reader.Update()

    actor = vtk.vtkActor()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor.SetMapper(mapper)

    return actor


def get_transforms():
    gps_reader = GPSReader(GPS_FILE)
    gps_data = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(gps_data)
    return imu_transforms


def export_scene_vrml(win, output_file):
    writer = vtk.vtkVRMLExporter()
    writer.SetInput(win)
    writer.SetFileName(output_file)
    writer.Write()


class Blockworld:

    def __init__(self):
        self.start = EXPORT_START
        self.end = EXPORT_START + EXPORT_NUM * EXPORT_STEP
        self.step = EXPORT_STEP
        self.count = 0

        self.ren = vtk.vtkRenderer()

        ''' Transforms '''

        self.imu_transforms = get_transforms()
        self.trans_wrt_imu = self.imu_transforms[self.start:self.end:self.step,
                0:3, 3]
        self.params = LoadParameters('q50_4_3_14_params')
        self.radar_params = self.params['radar']
        self.lidar_params = self.params['lidar']

        ''' Radar '''

        self.rdr_pts = loadRDRCamMap(MAP_FILE)
        self.radar_actors = []

        print 'Adding transforms'

        gps_cloud = VtkPointCloud(self.trans_wrt_imu[:, 0:3],
                0 * self.trans_wrt_imu[:, 0])
        self.ren.AddActor(gps_cloud.get_vtk_cloud())

        #print 'Adding octomap'

        #octomap_actor = load_octomap(OCTOMAP_H5_FILE)
        #self.ren.AddActor(octomap_actor)

        print 'Adding point cloud'

        cloud_actor = load_vtk_cloud(STATIC_VTK_FILE)
        self.ren.AddActor(cloud_actor)

        #dynamic_actor = load_vtk_cloud(DYNAMIC_VTK_FILE)
        #dynamic_actor.GetProperty().SetColor(0, 0, 1)
        #dynamic_actor.GetMapper().ScalarVisibilityOff()
        #self.ren.AddActor(dynamic_actor)

        print 'Adding car'

        self.car = load_ply('gtr.ply')
        self.ren.AddActor(self.car)

        print 'Rendering'

        self.ren.ResetCamera()

        self.win = vtk.vtkRenderWindow()
        self.win.AddRenderer(self.ren)
        self.win.SetSize(400, 400)

        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren.SetRenderWindow(self.win)
        mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(mouseInteractor)

        self.iren.Initialize()

        # Whether to write video

        self.record = False

        # Set up time

        self.iren.AddObserver('TimerEvent', self.update)
        self.timer = self.iren.CreateRepeatingTimer(100)

        # Add keypress event
        self.iren.AddObserver('KeyPressEvent', self.keyhandler)
        self.mode = 'ahead'

        self.iren.Start()

    def getCameraPosition(self):
        t = self.start + self.step * self.count
        if self.mode == 'ahead':
            position = self.imu_transforms[t, 0:3, 3]
            focal_point = self.imu_transforms[t + self.step, 0:3, 3]
        elif self.mode == 'behind':
            # FIXME Tune this
            position = self.imu_transforms[t - 5*self.step, 0:3, 3]
            focal_point = self.imu_transforms[t - 4*self.step, 0:3, 3]
        elif self.mode == 'above':
            position = self.imu_transforms[t - self.step, 0:3, 3] + np.array([0, 0, 200.0])
            focal_point = self.imu_transforms[t, 0:3, 3]
        elif self.mode == 'passenger':
            # TODO Not sure being inside mesh works...
            pass
        return position, focal_point

    def keyhandler(self, obj, event):
        key = obj.GetKeySym()
        if key == 'a':
            self.mode = 'above'
        elif key == 'b':
            self.mode = 'behind'
        elif key == 'd':
            self.mode = 'ahead'
        elif key == '0':
            self.count = 0
        elif key == 'r':
            if self.record:
                self.closeVideo()
                self.record = False
            else:
                self.startVideo()
                self.record = True
        else:
            pass

    def updateRadar(self):
        # Taken from testDrawRadarOnMap.py
        fren = self.iren.GetRenderWindow().GetRenderers().GetFirstRenderer()
        t = self.start + self.step * self.count
        radar_data = loadRDR(self.rdr_pts[t])[0]

        if radar_data.shape[0] > 0:
            #Convert from radar to lidar ref-frame
            radar_data[:, :3] = calibrateRadarPts(radar_data[:, :3], self.radar_params)

            #Convert from lidar to IMU ref-frame
            radar_data[:, :3] = np.dot(self.lidar_params['T_from_l_to_i'][:3, :3],
                radar_data[:, :3].transpose()).transpose()

            h_radar_data = np.hstack((radar_data[:, :3], np.ones((radar_data.shape[0], 1))))

            radar_data[:, :3] = np.dot(self.imu_transforms[t],
                h_radar_data.transpose()).transpose()[:, :3]

            for i in xrange(len(self.radar_actors)):
                fren.RemoveActor(self.radar_actors[i])

            self.radar_actors = []
            self.radar_clouds = []

            for i in xrange(radar_data.shape[0]):
                self.radar_clouds.append(VtkBoundingBox(radar_data[i, :]))
                (ax, ay, az) = euler_from_matrix(self.imu_transforms[t])
                box = self.radar_clouds[i].get_vtk_box(rot=az*180/np.pi)
                self.radar_actors.append(box)
                fren.AddActor(self.radar_actors[i])

    def update(self, iren, event):
        # Transform the car

        t = self.start + self.step * self.count
        imu_transform = self.imu_transforms[t, :, :]
        transform = vtk_transform_from_np(imu_transform)
        transform.RotateZ(90)
        transform.Translate(-2, -3, -2)
        self.car.SetUserTransform(transform)

        # Add the radar

        #self.updateRadar()

        # Set camera position
        fren = iren.GetRenderWindow().GetRenderers().GetFirstRenderer()
        cam = fren.GetActiveCamera()
        position, focal_point = self.getCameraPosition()
        cam.SetPosition(position)
        cam.SetFocalPoint(focal_point)
        cam.SetViewUp(0, 0, 1)
        fren.ResetCameraClippingRange()
        cam.SetClippingRange(0.1,1600)

        iren.GetRenderWindow().Render()

        if self.record:
            self.writeVideo()

        self.count += 1

    def startVideo(self):
        self.win2img = vtk.vtkWindowToImageFilter()
        self.win2img.SetInput(self.win)
        self.videoWriter = vtk.vtkFFMPEGWriter()
        self.videoWriter.SetFileName('/home/zxie/Desktop/blockworld.avi')
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
