import math
import numpy 
import colorsys

def heatColorMap(val, minc, maxc):
    rv = float(val) / (maxc - minc) # map to [0,1]
    h = (1-rv) * 1
    s = 1
    l = rv*0.5

    r,g,b = colorsys.hls_to_rgb(h,l,s)
    return 255*numpy.array([r,g,b])

