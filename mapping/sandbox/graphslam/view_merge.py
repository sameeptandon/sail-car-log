import sys
import vtk
from blockworld import load_vtk_cloud
from pipeline_config import DATA_DIR
import h5py

recolored = False

def keyhandler(obj, event):
    global recolored

    key = obj.GetKeySym()
    if key == 't':
        if not recolored:
            cloud_actor1.GetProperty().SetColor(0, 0, 1)
            cloud_actor1.GetMapper().ScalarVisibilityOff()
            cloud_actor2.GetProperty().SetColor(0, 1, 0)
            cloud_actor2.GetMapper().ScalarVisibilityOff()
            recolored = True
        else:
            cloud_actor1.GetMapper().ScalarVisibilityOn()
            cloud_actor2.GetMapper().ScalarVisibilityOn()
            recolored = False


if __name__ == '__main__':

    ren = vtk.vtkRenderer()
    cloud_actor1 = load_vtk_cloud(sys.argv[1])
    cloud_actor2 = load_vtk_cloud(sys.argv[2])
    transform_file = None
    if (len(sys.argv) > 3):
        transform_file = sys.argv[3]
        h5f = h5py.File(transform_file)
        T = h5f['transform'][...]
        print T
        transform = vtk.vtkTransform()
        transform.Translate(T[0, 3], T[1, 3], T[2, 3])
        cloud_actor2.SetUserTransform(transform)

    ren.AddActor(cloud_actor1)
    ren.AddActor(cloud_actor2)

    axes = vtk.vtkAxesActor()
    axes.AxisLabelsOff()
    ren.AddActor(axes)

    ren.ResetCamera()

    win = vtk.vtkRenderWindow()
    win.AddRenderer(ren)
    win.SetSize(800, 800)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(win)
    mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    iren.SetInteractorStyle(mouseInteractor)

    iren.Initialize()
    iren.AddObserver('KeyPressEvent', keyhandler)
    iren.Start()
