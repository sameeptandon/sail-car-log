import numpy 
import colorsys
from matplotlib.colors import hsv_to_rgb


def heatColorMapFast(vals, minc, maxc):
    rv = 1.0/(maxc - minc) * vals;
    H = rv*0.4
    S = 0*rv + 1
    V = 0*rv + 1.0
    HSV = numpy.dstack((H,S,V))
    output = hsv_to_rgb(HSV)
    R = output[0,:,0]
    G = output[0,:,1]
    B = output[0,:,2]
    return 255*numpy.dstack((B,G,R))


def heatColorMap(val, minc, maxc):
    rv = float(val) / (maxc - minc) # map to [0,1]
    h = (1-rv) * 1
    s = 1
    l = rv*0.5

    r,g,b = colorsys.hls_to_rgb(h,l,s)
    return 255*numpy.array([r,g,b])

def intensity2Color(dist, max_intensity = 255.0):
    # given a distance and a maximum distance, gives a color code for the distance.
    # red being closest, green is mid-range, blue being furthest

    blue = numpy.array([255,0,0])
    green = numpy.array([0,255,0])
    red = numpy.array([0,0,255])
    alpha = (dist/max_intensity)
    color = numpy.zeros([len(dist),3],dtype='u1')
    for i in range(len(dist)):
        if alpha[i]<0.5:
            color[i,:] = red*(1-alpha[i]*2)+green*alpha[i]*2
        else:
            beta = alpha[i]-0.5
            color[i,:] = green*(1-beta*2)+blue*beta*2
    return color

