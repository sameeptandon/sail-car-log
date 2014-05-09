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


R = np.eye(3)
R[1,0] = 0.5
R[0,1] = 0.5
R[2,2] = 0.001

L = np.linalg.cholesky(R)
print L
T = np.eye(4)
T[0:3,0:3] = L
T[0:3, 3] = [1.0, 0.0, 0.0]

print T

ellipsoid = VtkEllipsoid(T)
ren.AddActor(ellipsoid.get_vtk_ellipsoid())
# enable user interface interactor
iren.Initialize()
renWin.Render()
iren.Start()
