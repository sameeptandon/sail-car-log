from aivtk import aiWorld, aiRenderer, aiCloud
import numpy as np

def update(ren_interactor, event):
    # print 'hello'
    pass

if __name__ == '__main__':
    world = aiWorld((800, 400))
    world.update_cb = update

    cloud_ren = aiRenderer()
    cloud_ren.interactive = False

    world.add_renderer(cloud_ren)

    X = np.random.rand(100, 3)
    obj = aiCloud(X)
    cloud_ren.add_object(obj)

    world.start()
