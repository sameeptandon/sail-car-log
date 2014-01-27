import math
import numpy 
import colorsys
from matplotlib.colors import hsv_to_rgb


def heatColorMapFast(vals, minc, maxc):
    print vals.shape
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

