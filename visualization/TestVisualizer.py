from aivtk import *
import numpy as np

def update (renderers):
    cloud = renderers.cloud_ren.objects.clouds[0]
    box = renderers.cloud_ren.objects.boxes[0]
    # We can update the position of clouds and cubes by accessing their data
    # directly
    # cloud.data += np.array([.001] * 3)
    # box.data += np.array([.001] * 3)

if __name__ == '__main__':
    world = aiWorld((800, 400))
    # Use the default framerate (60 hz)
    world.update_cb = update

    # ex: Use a 30Hz framerate
    # world.update_cb = (update, 30)

    cloud_ren = aiRenderer()
    cloud_ren.ren.SetInteractive(False)
    def custom_handler (x, y, ai_obj, idx, default):
        print x, y
        print ai_obj, idx
        default()
    cloud_ren.mouse_handler.leftPress = custom_handler

    world.addRenderer(cloud_ren = cloud_ren)

    clouds = []
    num_pts = 100
    for i in xrange(3):
        pts = np.random.rand(num_pts, 3)
        cloud = aiCloud(pts)
        colors = (np.random.rand(*pts.shape) * 255).astype(np.uint8)
        cloud.color = colors
        clouds.append(cloud)

    # create a box witih the given bounds [xmin,xmax,ymin,ymax,zmin,zmax]
    box = aiBox((0, 1, 0, 1, 0, 1))
    box.wireframe = True
    box.color = np.array((255, 10, 255))

    car = aiPly('gtr.ply')

    cloud_ren.addObjects(clouds=clouds, boxes=box)
    # We can add objects to the renderer later
    # cloud_ren.addObjects(car = car)

    # We can add and delete objects as lists
    # cloud_ren.removeObjects(cloud_ren.objects.clouds[:2])

    # Access objects through their renderer by the name given in addObjects
    cloud_ren.objects.boxes[0].opacity = .3
    cloud_ren.objects.boxes[0].wireframe = True

    # Make sure to start the world (runs the update function)
    world.start()
