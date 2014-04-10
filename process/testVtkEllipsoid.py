import vtk
from VtkRenderer import *
import numpy as np

# create a rendering window and renderer
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)
WIDTH=640
HEIGHT=480
renWin.SetSize(WIDTH,HEIGHT)

# create a renderwindowinteractor
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)
axes = vtk.vtkAxesActor()
#axes.AxisLabelsOff()
ren.AddActor(axes)


T = np.eye(4)
T[1,0] = 0.5
T[0,1] = 0.5
T[2,2] = 0.001

L = np.linalg.cholesky(T)
print T
print L

ellipsoid = VtkEllipsoid(L)
ren.AddActor(ellipsoid.get_vtk_ellipsoid())
# enable user interface interactor
iren.Initialize()
renWin.Render()
iren.Start()
