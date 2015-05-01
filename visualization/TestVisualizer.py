from aivtk import aiWorld, aiRenderer, aiCloud, aiBox
import numpy as np

def update(ren_interactor, event):
    # print 'hello'
    pass

if __name__ == '__main__':
    world = aiWorld((800, 400))
    world.update_cb = update

    cloud_ren = aiRenderer()
    cloud_ren.ren.SetInteractive(False)

    world.addRenderer(cloud_ren)

    pts = np.random.rand(100, 3)
    cloud = aiCloud(pts)
    colors = np.random.rand(*pts.shape) * 255
    colors = colors.astype(np.uint8)
    cloud.color = colors

    box = aiBox((0, 1, 0, 1, 0, 1))
    box.wireframe = True
    box.color = np.array((255, 10, 255))

    cloud_ren.addObject(cloud)
    cloud_ren.addObject(box)

    world.start()
