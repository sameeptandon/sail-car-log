import vtk
from numpy import random
import numpy as np
import vtk.util.numpy_support as converter
import time
import cv2
import itertools


class VtkImage:
    def __init__(self, im):
        self.im =  cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    def get_vtk_image(self):
        importer = vtk.vtkImageImport()
        importer.SetDataSpacing(1,1,1)
        importer.SetDataOrigin(0,0,0)
        importer.SetWholeExtent(0, self.im.shape[1] - 1, 
                0, self.im.shape[0] - 1, 0, 0)
        importer.SetDataExtentToWholeExtent()
        importer.SetDataScalarTypeToUnsignedChar()
        importer.SetNumberOfScalarComponents(self.im.shape[2])
        importer.SetImportVoidPointer(self.im)
        importer.Update()
        flipY = vtk.vtkImageFlip()
        flipY.SetFilteredAxis(1)
        flipY.SetInputConnection(importer.GetOutputPort())
        flipY.Update()
        yActor = vtk.vtkImageActor()
        yActor.SetInput(flipY.GetOutput())

        return yActor

class VtkEllipsoid:
    def __init__(self, T):
        self.T = (T[0,0], T[0,1], T[0,2], T[0,3],
                  T[1,0], T[1,1], T[1,2], T[1,3],
                  T[2,0], T[2,1], T[2,2], T[2,3],
                  T[3,0], T[3,1], T[3,2], T[3,3])


    def get_vtk_ellipsoid(self):
        self.transformMatrix = vtk.vtkMatrix4x4()
        self.transformMatrix.DeepCopy(self.T)
        transform = vtk.vtkTransform()
        transform.SetMatrix(self.transformMatrix)

        self.source = vtk.vtkSphereSource()
        self.source.SetRadius(1.0)
        self.source.SetCenter(0.0,0.0,0.0)

        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        transformFilter.SetInputConnection(self.source.GetOutputPort())
        transformFilter.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transformFilter.GetOutputPort())
        
        actor = vtk.vtkActor()
        actor.GetProperty().SetOpacity(0.8)
        actor.SetMapper(mapper)

        # assign actor to the renderer
        return actor





class VtkBoundingBox:
    def __init__(self, properties):
        # (x, y) is the center-back of the car
        (x, y, z, l, w) = tuple(properties[:5])
        h = 1
        self.bounds = (x, x+l, y-w/2., y+w/2., z-h/2., z+h/2.)

    def get_vtk_box(self, rot = 0):
        # create source
        source = vtk.vtkCubeSource()
        source.SetBounds(self.bounds)

        # mapper
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())

        # actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetRepresentationToWireframe()
        actor.GetProperty().SetLineWidth(1)
        actor.GetProperty().LightingOff()

        actor.SetOrigin(source.GetCenter())
        actor.RotateZ(rot)

        # assign actor to the renderer
        return actor

"""
Uses VTK to render a point cloud based on intensities, which might be floating point numbers or RGB values. 

Disclaimer: This function passes points to vtk, so make sure that your data does not get deallocated by python. Somethings are copied too. It's not really an efficient function and it's a slight miracle that it even works. 

For internal VTK debugging, here's the layout of things: 
    - vtkPoints consist of the x,y,z coordinates. Pass in an array of size m x 3 (where m is the number of points) 
    - vtkCells tells vtk how to render the points. It is formatted as "1 1 1 2 1 3 ... 1 m' where the 1 tells how many points it should consider in a surface and the even element says which point id to use.

The function build_vtk_polydata will do the assembling magic.

Then you can call get_vtk_color_cloud or get_vtk_cloud based on how you want to color map each point. 
"""

class VtkPointCloud:
    def __init__(self, xyz, intensity):
        self.xyz = np.ascontiguousarray(xyz)
        self.intensity = np.ascontiguousarray(intensity)
        
        num_points = self.xyz.shape[0]

        np_cells_A = np.ones(num_points,dtype=np.int64)
        np_cells_B = np.arange(0,num_points,dtype=np.int64)
        self.np_cells = np.empty(2*num_points,dtype=np.int64)
        self.np_cells[::2] = np_cells_A
        self.np_cells[1::2] = np_cells_B

    def build_vtk_polydata(self):
        vtkPolyData = vtk.vtkPolyData()
        vtkPoints = vtk.vtkPoints()
        vtkCells = vtk.vtkCellArray()
        vtkPolyData.SetPoints(vtkPoints)
        vtkPolyData.SetVerts(vtkCells)
        num_points = self.xyz.shape[0]
        
        vtk_data = converter.numpy_to_vtk(self.xyz)
        vtkPoints.SetNumberOfPoints(num_points)
        vtkPoints.SetData(vtk_data)
      
        vtkCells.SetCells(num_points, converter.numpy_to_vtkIdTypeArray(self.np_cells, deep=1))

        return (vtkPolyData, vtkPoints, vtkCells)

    def get_vtk_color_cloud(self):
        assert(self.intensity.shape[1] == 3)
        (vtkPolyData, vtkPoints, vtkCells) = self.build_vtk_polydata()
        self.intensity = self.intensity.astype(np.uint8)
        vtk_color_data = converter.numpy_to_vtk(self.intensity)
        vtk_color_data.SetName('ColorArray')
        vtkPolyData.GetPointData().SetScalars(vtk_color_data)
        vtkPolyData.GetPointData().SetActiveScalars('ColorArray')
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInput(vtkPolyData)
        vtkActor = vtk.vtkActor()
        vtkActor.SetMapper(mapper)
      
        return vtkActor


    def get_vtk_cloud(self, zMin=-10.0,zMax=10.0):
        assert( len(self.intensity.shape) == 1)
        (vtkPolyData, vtkPoints, vtkCells) = self.build_vtk_polydata()

        self.intensity == self.intensity.astype(np.float32)
        vtk_intensity_data = converter.numpy_to_vtk(self.intensity)
        vtk_intensity_data.SetName('DepthArray')
        vtkPolyData.GetPointData().SetScalars(vtk_intensity_data)
        
        num_points = self.xyz.shape[0]
        #vtkDepth = vtk.vtkFloatArray()
        #vtkDepth.SetName('DepthArray')
        #vtkPolyData.GetPointData().SetScalars(vtkDepth)
        #vtkDepth.SetVoidArray(self.intensity, num_points, 1)
        vtkPolyData.GetPointData().SetActiveScalars('DepthArray')

        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInput(vtkPolyData)
        mapper.SetColorModeToDefault()
        mapper.SetScalarRange(zMin, zMax)
        mapper.SetScalarVisibility(1)
        vtkActor = vtk.vtkActor()
        vtkActor.SetMapper(mapper)
      
        return vtkActor


############# sample callback setup ###############
class vtkTimerCallback():
   def __init__(self):
     pass
 
   def execute(self,obj,event):
     t = time.time()
     data = 40*(random.random((60000,3))-0.5)
     pointCloud = VtkPointCloud(data, data[:,2])
     iren = obj
     iren.GetRenderWindow().GetRenderers().GetFirstRenderer().RemoveActor(self.actor)
     self.actor = pointCloud.get_vtk_cloud()
     iren.GetRenderWindow().GetRenderers().GetFirstRenderer().AddActor(self.actor)
     iren.GetRenderWindow().Render()
     print time.time() - t

if __name__ == '__main__': 

  data = 40*(random.random((600,3))-0.5)
  pointCloud = VtkPointCloud(data, data[:,2])
  actor = pointCloud.get_vtk_cloud()

  # Renderer
  renderer = vtk.vtkRenderer()
  renderer.AddActor(actor)
  renderer.SetBackground(0.0, 0.0, 0.)
  renderer.ResetCamera()
  
  # Render Window
  renderWindow = vtk.vtkRenderWindow()
  renderWindow.SetSize(600,600)
  renderWindow.AddRenderer(renderer)
  
  # Interactor
  renderWindowInteractor = vtk.vtkRenderWindowInteractor()
  renderWindowInteractor.SetRenderWindow(renderWindow)
  
  # Begin Interaction
  renderWindow.Render()
  renderWindowInteractor.Initialize()
  
  cb = vtkTimerCallback()
  cb.actor = actor 
  renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
  timerId = renderWindowInteractor.CreateRepeatingTimer(50)
  
  renderWindowInteractor.Start()
