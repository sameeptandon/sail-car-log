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

    clouds = []
    num_pts = 100
    for i in xrange(3):
        pts = np.random.rand(num_pts, 3)
        cloud = aiCloud(pts)
        colors = (np.random.rand(*pts.shape) * 255).astype(np.uint8)
        cloud.color = colors
        clouds.append(cloud)

    box = aiBox((0, 1, 0, 1, 0, 1))
    box.wireframe = True
    box.color = np.array((255, 10, 255))

    cloud_ren.addObjects(clouds=clouds, boxes=box)

    cloud_ren.objects.boxes[0].color = (10, 0, 100)
    cloud_ren.objects.clouds[0].color = (np.ones((num_pts, 3)) * 255).astype(np.uint8)

    world.start()
