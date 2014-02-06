from VtkRenderer import *
from VideoReader import *
import cv2, cv, sys

global count
count = 0

class ImageGrabberCallback:

    def __init__(self, reader):
        self.reader = reader

    def execute(self, iren, event):
        global count
        t = time.time()
        (success, image) = reader.getNextFrame()
        print 'read: ', time.time() - t
        t = time.time()
        if not success:
            return
        self.VtkImage = VtkImage(image)
        actor = self.VtkImage.get_vtk_image()
        print 'convert:', time.time() -t 
        t = time.time()

        count += 1
        if count > 1:
            iren.GetRenderWindow().GetRenderers().GetFirstRenderer().RemoveActor(self.actor)
        self.actor = actor
        iren.GetRenderWindow().GetRenderers().GetFirstRenderer().AddActor(self.actor)
        if count == 1:
            iren.GetRenderWindow().GetRenderers().GetFirstRenderer().ResetCamera()
            iren.GetRenderWindow().GetRenderers().GetFirstRenderer().GetActiveCamera().Zoom(1.6)

        iren.GetRenderWindow().Render()
        print 'render:', time.time() - t

if __name__ == '__main__':
    reader = VideoReader(sys.argv[1])


    renderWindow = vtk.vtkRenderWindow()
    renderWindow.SetSize(1280/2, 960/2)
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)

    renderer = vtk.vtkRenderer()
    renderWindow.AddRenderer(renderer)

    renderWindow.Render()
    cb = ImageGrabberCallback(reader)
    interactor.AddObserver('TimerEvent', cb.execute)
    timerId = interactor.CreateRepeatingTimer(1)
    interactor.Start()
