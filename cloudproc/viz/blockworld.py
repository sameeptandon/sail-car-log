import numpy as np
from numpy.linalg import inv
import vtk
#import vtk.util.numpy_support as converter
from vtk.io import vtkPLYReader
import h5py
from pipeline_config import COLOR_OCTOMAP_H5_FILE, MERGED_CLOUD_FILE,\
        GPS_FILE, EXPORT_START, EXPORT_STEP, EXPORT_NUM, OCTOMAP_SINGLE_FILES,\
        OCTOMAP_H5_FILE
from VtkRenderer import VtkPointCloud

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

        print 'Adding transforms'

        gps_cloud = VtkPointCloud(self.trans_wrt_imu[:, 0:3],
                -1 * self.trans_wrt_imu[:, 0])
        self.ren.AddActor(gps_cloud.get_vtk_cloud())

        #print 'Adding octomap'

        #octomap_actor = load_octomap(OCTOMAP_H5_FILE)
        #self.ren.AddActor(octomap_actor)

        print 'Adding point cloud'

        cloud_actor = load_vtk_cloud(MERGED_CLOUD_FILE.replace('.pcd', '.vtk'))
        self.ren.AddActor(cloud_actor)

        print 'Adding car'

        self.car = load_ply('gtr.ply')
        self.ren.AddActor(self.car)

        print 'Rendering'

        self.ren.ResetCamera()

        self.win = vtk.vtkRenderWindow()
        self.win.AddRenderer(self.ren)
        self.win.SetSize(400, 400)

        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren .SetRenderWindow(self.win)
        mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(mouseInteractor)

        self.iren.Initialize()

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
            roll0 = 80
        elif self.mode == 'behind':
            position = self.imu_transforms[t - self.step, 0:3, 3]
            focal_point = self.imu_transforms[t, 0:3, 3]
            roll0 = 80
        elif self.mode == 'above':
            position = self.imu_transforms[t - self.step, 0:3, 3] + np.array([0, 0, 100.0])
            focal_point = self.imu_transforms[t, 0:3, 3]
            roll0 = 0
        elif self.mode == 'passenger':
            pass
        return position, focal_point, roll0

    def keyhandler(self, obj, event):
        key = obj.GetKeySym()
        if key == 'a':
            self.mode = 'above'
        elif key == 'b':
            self.mode == 'behind'
        elif key == 'd':
            self.mode = 'ahead'
        else:
            pass

    def update(self, iren, event):
        # Transform the car
        imu_transform = self.imu_transforms[self.start + self.step * self.count, :, :]
        transform = vtk_transform_from_np(imu_transform)
        transform.RotateZ(90)
        transform.Translate(-2, 3, -1)
        self.car.SetUserTransform(transform)

        fren = iren.GetRenderWindow().GetRenderers().GetFirstRenderer()
        cam = fren.GetActiveCamera()
        position, focal_point, roll0 = self.getCameraPosition()
        cam.SetPosition(position)
        cam.SetFocalPoint(focal_point)
        if self.count == 0:
            cam.Roll(roll0)

        iren.GetRenderWindow().Render()
        self.count += 1


if __name__ == '__main__':
    blockworld = Blockworld()
