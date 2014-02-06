import vtk
from numpy import random
import numpy as np
import vtk.util.numpy_support as converter
import time
import cv2


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



class VtkPointCloud:
    def __init__(self, xyz, intensity):
        self.xyz = np.ascontiguousarray(xyz)
        self.intensity = intensity.ravel()

        num_points = self.xyz.shape[0]

        np_cells_A = np.ones(num_points,dtype=np.int64)
        np_cells_B = np.arange(0,num_points,dtype=np.int64)
        self.np_cells = np.empty(2*num_points,dtype=np.int64)
        self.np_cells[::2] = np_cells_A
        self.np_cells[1::2] = np_cells_B

    def get_vtk_cloud(self, zMin=-10.0,zMax=10.0):

        vtkPolyData = vtk.vtkPolyData()
        vtkPoints = vtk.vtkPoints()
        vtkCells = vtk.vtkCellArray()
        vtkDepth = vtk.vtkFloatArray()
        vtkDepth.SetName('DepthArray')
        vtkPolyData.SetPoints(vtkPoints)
        vtkPolyData.SetVerts(vtkCells)
        vtkPolyData.GetPointData().SetScalars(vtkDepth)
        vtkPolyData.GetPointData().SetActiveScalars('DepthArray')
      
        num_points = self.xyz.shape[0]
        
        vtk_data = converter.numpy_to_vtk(self.xyz)
        vtkPoints.SetNumberOfPoints(num_points)
        vtkPoints.SetData(vtk_data)
      
        vtkCells.SetCells(num_points, converter.numpy_to_vtkIdTypeArray(self.np_cells, deep=1))
        vtkDepth.SetVoidArray(self.intensity, num_points, 1)
      
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
