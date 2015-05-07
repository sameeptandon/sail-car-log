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

def intensity2Color(dist, max_intensity = 255.0, bgr=True):
    # given an intensity and a maximum value, gives a color code for the intensity.
    # red being lowest, green is mid-range, blue being highest
    green = numpy.array([0,255,0])
    if bgr:
      blue = numpy.array([255,0,0])
      red = numpy.array([0,0,255])
    else:
      blue = numpy.array([0,0,255])
      red = numpy.array([255,0,0])
    
    alpha = (dist/max_intensity)
    num_channels = 3
    color = numpy.zeros([len(dist),num_channels],dtype='u1')

    for c in range(num_channels):
      idx1 = numpy.where(alpha<0.5)[0]
      idx2 = numpy.where(alpha>=0.5)[0]
      color[idx1,c] = red[c]*(1-alpha[idx1]*2)+green[c]*alpha[idx1]*2
      beta = alpha[idx2]-0.5
      color[idx2,c] = green[c]*(1-beta*2)+blue[c]*beta*2
    print color
    return color
