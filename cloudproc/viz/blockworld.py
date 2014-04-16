import numpy as np
from numpy.linalg import inv
import vtk
import vtk.util.numpy_support as converter
from vtk.io import vtkPLYReader
import h5py
from pipeline_config import COLOR_OCTOMAP_H5_FILE, MERGED_CLOUD_FILE,\
        GPS_FILE, EXPORT_START, EXPORT_STEP, EXPORT_NUM
from VtkRenderer import VtkPointCloud

from GPSTransforms import IMUTransforms
from GPSReader import GPSReader
from Q50_config import LoadParameters


def load_ply(ply_file):
    reader = vtkPLYReader()
    reader.SetFileName('g35.ply')
    reader.Update()
    reader.Update()

    ply_mapper = vtk.vtkPolyDataMapper()
    ply_mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(ply_mapper)
    return actor


def load_octomap(octomap_h5_file, conf=0.9, wireframe=False):
    h5f = h5py.File(octomap_h5_file, 'r')
    octree_data = h5f['octree'][...]

    octree_data = octree_data[octree_data[:, 4] > conf]
    pts = vtk.vtkPoints()
    #vtk_pt_data = converter.numpy_to_vtk(np.ascontiguousarray(octree_data[:, 0:3]))
    #pts.SetData(vtk_pt_data)

    colors = vtk.vtkUnsignedCharArray()
    colors.SetNumberOfComponents(3)
    #color_data = np.ascontiguousarray(octree_data[:, 5:8])
    #colors = converter.numpy_to_vtk(color_data)
    #colors.SetName('ColorArray')
    #polydata.GetPointData().SetActiveScalars('ColorArray')

    for k in range(octree_data.shape[0]):
        pts.InsertNextPoint(*octree_data[k, 0:3])
        r = int(octree_data[k, 5])
        g = int(octree_data[k, 6])
        b = int(octree_data[k, 7])
        colors.InsertNextTupleValue((r, g, b))

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(pts)
    polydata.GetPointData().SetScalars(colors)

    cube = vtk.vtkCubeSource()
    cube.SetXLength(octree_data[0, 3])
    cube.SetYLength(octree_data[0, 3])
    cube.SetZLength(octree_data[0, 3])

    glyph = vtk.vtkGlyph3D()
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


if __name__ == '__main__':
    start = EXPORT_START
    end = EXPORT_START + EXPORT_NUM * EXPORT_STEP
    step = EXPORT_STEP

    ren = vtk.vtkRenderer()

    ''' Transforms '''

    imu_transforms = get_transforms()
    trans_wrt_imu = imu_transforms[start:end:step, 0:3, 3]

    params = LoadParameters('q50_4_3_14_params')
    T_from_i_to_l = inv(params['lidar']['T_from_l_to_i'])
    T_from_i_to_l[0:3, 0:3] = np.eye(3)
    ones = np.ones((trans_wrt_imu.shape[0], 1))
    trans_wrt_imu_homog = np.hstack((trans_wrt_imu, ones))
    lidar_origins = np.dot(T_from_i_to_l, trans_wrt_imu_homog.T).T[:, 0:3]
    print lidar_origins.shape

    print 'Adding transforms'

    gps_cloud = VtkPointCloud(trans_wrt_imu[:, 0:3], -1 * trans_wrt_imu[:, 0])
    lidar_origin_cloud = VtkPointCloud(lidar_origins, 0 * trans_wrt_imu[:, 0])
    ren.AddActor(gps_cloud.get_vtk_cloud())
    ren.AddActor(lidar_origin_cloud.get_vtk_cloud())


    print 'Adding octomap'

    octomap_actor = load_octomap(COLOR_OCTOMAP_H5_FILE)
    ren.AddActor(octomap_actor)

    print 'Adding point cloud'

    #cloud_actor = load_vtk_cloud(MERGED_CLOUD_FILE.replace('.pcd', '.vtk'))
    #ren.AddActor(cloud_actor)

    print 'Adding car'
    g35 = load_ply('g35.ply')
    ren.AddActor(g35)

    print 'Displaying stuff'

    ren.ResetCamera()

    win = vtk.vtkRenderWindow()
    win.AddRenderer(ren)
    win.SetSize(400, 400)

    iren = vtk.vtkRenderWindowInteractor()
    iren .SetRenderWindow(win)
    mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    iren.SetInteractorStyle(mouseInteractor)

    win.Render()
    iren.Initialize()
    iren.Start()

    #print 'Exporting scene'
    #export_scene_vrml(win, 'test.wrl')
