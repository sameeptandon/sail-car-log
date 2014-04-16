import numpy as np
import vtk
import vtk.util.numpy_support as converter
import h5py
from pipeline_config import OCTOMAP_H5_FILE, MERGED_CLOUD_FILE
from VtkRenderer import VtkPointCloud

class VtkCube:
    def __init__(self, properties):
        (x, y, z, s) = tuple(properties[:4])
        self.bounds = (x - s/2.0, x + s/2.0, y - s/2.0, y + s/2.0, z - s/2.0, z + s/2.0)

    def getActor(self, rot=0):
        # create source
        source = vtk.vtkCubeSource()
        source.SetBounds(self.bounds)

        ## mapper
        #mapper = vtk.vtkPolyDataMapper()
        #mapper.SetInputConnection(source.GetOutputPort())

        ## actor
        #actor = vtk.vtkActor()
        #actor.SetMapper(mapper)
        ##actor.GetProperty().SetRepresentationToWireframe()
        ##actor.GetProperty().SetLineWidth(1)
        #actor.GetProperty().LightingOff()

        #actor.SetOrigin(source.GetCenter())
        #actor.RotateZ(rot)

        ## assign actor to the renderer
        #return actor
        return source.GetOutput()


if __name__ == '__main__':
    reader = vtk.vtkDataSetReader()
    reader.SetFileName(MERGED_CLOUD_FILE.replace('.pcd', '.vtk'))
    reader.Update()
    #import IPython; IPython.embed()

    h5f = h5py.File(OCTOMAP_H5_FILE, 'r')
    octree_data = h5f['octree'][...]
    #print octree_data.shape

    ren = vtk.vtkRenderer()

    octree_data = octree_data[octree_data[:, 4] > 0.9]
    #print octree_data[:, 0:3]
    #print octree_data.shape
    pts = vtk.vtkPoints()
    #cloud = VtkPointCloud(octree_data[:, 0:3], np.ones(octree_data.shape[0]))
    #vtk_pt_data = converter.numpy_to_vtk(np.ascontiguousarray(octree_data[:, 0:3]))
    #pts.SetData(vtk_pt_data)
    colors = vtk.vtkUnsignedCharArray()
    colors.SetNumberOfComponents(3)
    for k in range(octree_data.shape[0]):
        pts.InsertNextPoint(*octree_data[k, 0:3])
        r = int(octree_data[k, 5])
        g = int(octree_data[k, 6])
        b = int(octree_data[k, 7])
        colors.InsertNextTupleValue((r, g, b))

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(pts)

    print octree_data[:, 5:8]
    #color_data = np.ascontiguousarray(octree_data[:, 5:8])
    #print color_data
    #colors = converter.numpy_to_vtk(color_data)
    #colors.SetName('ColorArray')
    print colors
    polydata.GetPointData().SetScalars(colors)
    #polydata.GetPointData().SetActiveScalars('ColorArray')

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
    #actor.GetProperty().SetRepresentationToWireframe()
    #actor.GetProperty().SetLineWidth(1)
    actor.GetProperty().LightingOff()
    #actor.GetProperty().SetOpacity(0.2)
    ren.AddActor(actor)
    #ren.AddActor(cloud.get_vtk_cloud())

    actor = vtk.vtkActor()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor.SetMapper(mapper)
    #ren.AddActor(actor)

    ren.ResetCamera()

    win = vtk.vtkRenderWindow()
    win.AddRenderer(ren)
    #iren = vtk.vtkRenderWindowInteractor()
    #iren.SetRenderWindow(win)
    win.SetSize(400, 400)

    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(win)
    mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    renderWindowInteractor.SetInteractorStyle(mouseInteractor)

    win.Render()
    renderWindowInteractor.Initialize()
    renderWindowInteractor.Start()
