import aivtk

def update(iren, event):
    print 'hello'

if __name__ == '__main__':
    world = aivtk.aiWorld((800, 400))
    world.update_cb = update

    cloud_ren = aivtk.aiRenderer()
    cloud_ren.color = (1, 0, 0)
    cloud_ren.position = (0, 0, .5, .5)

    world.add_renderer(cloud_ren)

    world.start()
