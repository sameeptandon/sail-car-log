import numpy as np
import cv2
import scipy.weave

def computeDistanceTransform(D, gamma, alpha):
    logD = np.log(D);
    logD = logD.astype(np.float32)
    logD = computeLogDistanceTransform(logD, gamma)
    F = np.exp(logD)
    return alpha*D + (1-alpha)*F

def computeLogDistanceTransformSlow(D, gamma): 
    # assume that D is logarithmic in the edges
    width = D.shape[0]
    height = D.shape[1]
    lg = np.log(gamma)

    for x in range(1,width):
        for y in range(1,height):
            D[x,y] = max(D[x,y], D[x-1,y]+lg, D[x,y-1]+lg, D[x-1,y-1]+lg)

    for x in reversed(range(width-1)):
        for y in reversed(range(height-1)):
            D[x,y] = max(D[x,y], D[x+1,y]+lg, D[x,y+1]+lg, D[x+1,y+1]+lg)
    
    #print D
    return D

def computeLogDistanceTransform(D, gamma): 
    # assume that D is logarithmic in the edges
    width = D.shape[0]
    height = D.shape[1]
    lg = np.log(gamma)
    code = \
    """
    using namespace std;
    for (int x = 1; x < width; x++) { 
        for (int y = 1; y < height; y++) {
            float l = lg; 
            float p1 = D(x,y);
            float p2 = D(x-1,y) + l;
            float p3 = D(x,y-1) + l;
            float p4 = D(x-1,y-1) + l;
            D(x,y) = max(p1,max(p2,max(p3,p4)));
        }
    }   

    for (int x = width-2; x >= 0 ; x--) { 
        for (int y = height-2; y >= 0; y--) {
            float l = lg; 
            float p1 = D(x,y);
            float p2 = D(x+1,y) + l;
            float p3 = D(x,y+1) + l;
            float p4 = D(x+1,y+1) + l;
            D(x,y) = max(p1,max(p2,max(p3,p4)));
        }
    }   
    """
    scipy.weave.inline(code, ['D', 'width', 'height', 'lg'], headers=['<algorithm>'], 
            type_converters=scipy.weave.converters.blitz)

    return D


def gauss_filt(sigma):
    # Isotropic
    w = 2 * int(np.ceil(sigma))
    G = np.array(xrange(-w, w + 1)) ** 2
    G = G.reshape((G.size, 1)) + G
    G = np.exp(-G / (2.0 * sigma * sigma))
    G /= np.sum(G)
    return G


def dgauss_filt(sigma):
    '''
    Generate a derivative of Gaussian filter in x (left-to-right)
    and y (top-to-bottom) directions
    '''
    G = gauss_filt(sigma)
    G_y, G_x = np.gradient(G)
    G_x *= 2.0 / np.sum(np.abs(G_x))
    G_y *= 2.0 / np.sum(np.abs(G_y))

    return G_x, G_y

def processImage(I):
    from scipy.signal import convolve2d
    
    E = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    G_x, G_y = dgauss_filt(1.0)
    I_x = -convolve2d(E, G_x, mode='same')
    I_y = -convolve2d(E, G_y, mode='same')
    I_mag = np.sqrt(I_x ** 2 + I_y ** 2) 
    edges = computeDistanceTransform(I_mag, 0.98, 1.0/2.0)
    return edges

