from LidarTransforms import * 
import sys, os
from VideoReader import *
from CameraParams import * 
import cv2
from cv2 import imshow, waitKey
from numpy.linalg import norm
from ColorMap import *
from numpy import exp, log, sqrt
from transformations import euler_matrix
import scipy.weave
import itertools
from ArgParser import * 

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
    lg = log(gamma)

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
    lg = log(gamma)
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


def generateEdgeFilterKernels(): 
    kernels = []
    for x in range(3):
        for y in range(3):
            K = np.zeros((3,3))
            K[1,1] = 10.0
            K[x,y] = -10.0
            if (x != 1 and y != 1):
                kernels.append(K)
    return kernels

def processPointCloud(raw_pts): 
    # add rotational angle and distance to pts
    pts = np.zeros((raw_pts.shape[0], raw_pts.shape[1]+2), dtype=np.float32)
    pts[:,:-2] = raw_pts
    pts[:,-2] = np.arctan2(pts[:,1], pts[:,0]) + np.pi
    pts[:,-1] = np.sqrt(np.sum( pts[:, 0:3] ** 2, axis=1 )) 

    pts = pts[ pts[:,5].argsort() ] # sort on rotational angle
    pts = pts[ pts[:,4].argsort(kind='mergesort') ] # stable sort on laser num


    pts[0,3] = 0.0;
    pts[-1,3] = 0.0
    """
    pts[1:-1,3] = np.maximum(pts[0:-2,-1] - pts[1:-1, -1],
                             pts[2:, -1] - pts[1:-1, -1])
    pts[1:-1,3] = np.maximum(pts[1:-1,3], 0)
    
    """
    for idx in range(1,pts.shape[0]-1):
        if pts[idx,4] == pts[idx-1,4] and pts[idx,4] == pts[idx+1,4]: 
            pts[idx,3] = max(pts[idx-1,-1] - pts[idx,-1], 
                             pts[idx+1,-1] - pts[idx,-1],
                             0)
        else:
            pts[idx,3] = 0.0
    pts = pts[pts[:,0] > 0, :]
    #pts = pts[pts[:,3] > 2.0, :]
    return pts

def computeReprojection(C, raw_pts, cam):
    pts = raw_pts[:, 0:3].copy()
    pts[:, 0] += C[0]
    pts[:, 1] += C[1]
    pts[:, 2] += C[2]
    R = euler_matrix(C[3], C[4], C[5])[0:3,0:3]
    pts_wrt_cam = np.dot(R, dot(R_to_c_from_l_old(cam), pts.transpose()))
    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    return (pix, pts_wrt_cam)

def computeMask(pix, pts_wrt_cam):
    width = 8
    mask = np.logical_and(True, pix[0,:] > 0 + width / 2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width / 2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width / 2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width / 2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    return mask

def computeReprojectionScore(C, pts, I, cam):
    (pix, pts_wrt_cam) = computeReprojection(C, pts, cam)
    mask = computeMask(pix, pts_wrt_cam)

    px = pix[1,mask]
    py = pix[0,mask]
    return np.sum(I[px,py])


def gridsearch(C, batch, cam):
    m = range(-1,2)
    step_t = 0.01
    step_r = 0.003

    best_score = -float("inf")
    best_d = None
    #scores = np.zeros((729,1))
    scores = np.zeros((3**3,1))
    idx = 0

    for delta in itertools.product(m, repeat=3):
        #(dtx, dty, dtz, drx, dry, drz) = delta
        #d = np.array([step_t*dtx, step_t*dty, step_t*dtz, step_r*drx, step_r*dry, step_r*drz])
        (drx, dry, drz) = delta
        d = step_r * np.array([drx, dry, drz])
        C_new = C.copy()
        C_new[3:6] += d
        score = 0
        for p in batch:
            E = p[2]
            proc_pts = p[3]
            score = score + computeReprojectionScore(C_new, proc_pts, E, cam)
        scores[idx] = score
        if score > best_score:
            best_score = score
            best_C = C_new.copy()
        if np.all(np.array(delta)) == 0:
            current_score = score 
        idx = idx + 1
    
    print scores
    print current_score
    if np.sum( scores > current_score ) >= 3**3 / 3**3:
        return (best_C, best_score)
    else:
        return (C, current_score)


def drawReprojection(C, pts, I, cam, colorMap=False):
    (pix, pts_wrt_cam) = computeReprojection(C, pts, cam)
    mask = computeMask(pix, pts_wrt_cam)

    px = pix[1,mask]
    py = pix[0,mask]
    if colorMap: 
        intensity = pts[mask, 3]
        colors = heatColorMapFast(intensity, 0, 255)
        I[px,py,:] = colors[0,:,:]
    else:
        I[px,py] = 255 
        for p in range(2):
            I[np.minimum(px+p,959),py] = 255 
            I[np.maximum(px-p,0),py] = 255
            I[px,np.minimum(py+p,1279)] = 255 
            I[px,np.maximum(py-p,0)] = 255
        
    imshow('display', I)
    waitKey(10)

def getNextData(reader, LDRFrameMap):
    for idx in range(25):
        (success, img) = reader.getNextFrame()
        if not success:
            reader.setFrame(5)
    ldr_frame = loadLDR(LDRFrameMap[reader.framenum])
    return (success, img, ldr_frame)
  

def processData(data):
    I, pts = data
    E = processImage(I);
    proc_pts = processPointCloud(pts)
    dist = np.sqrt(np.sum( proc_pts[:, 0:3] ** 2, axis = 1))
    proc_pts = proc_pts[ dist > 3, : ] 
    proc_pts = proc_pts[ proc_pts[:, 3] > 2.0, :]
    return [I, pts, E, proc_pts]

def processBatch(batch):
    processed = [ ]
    count = 0
    for data in batch:
        print 'Processing:', count, 'out of', len(batch)
        count += 1
        output = processData(data)
        processed.append(output)
    return processed

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
    G_x, G_y = dgauss_filt(4.0)
    I_x = -convolve2d(E, G_x, mode='same')
    I_y = -convolve2d(E, G_y, mode='same')
    I_mag = np.sqrt(I_x ** 2 + I_y ** 2) 
    edges = computeDistanceTransform(I_mag, 0.98, 1.0/2.0)
    return edges

"""
def processImage(I):
    kernels = generateEdgeFilterKernels()

    # convert the image to grayscale
    E = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)

    # run an edge filter
    edges = cv2.filter2D(E, cv2.CV_8U, np.zeros((1,1)))
    
    for k in kernels: 
        edges = np.maximum(edges, np.abs(cv2.filter2D(E, cv2.CV_8U, k)))

    edges = computeDistanceTransform(edges+1, 0.98, 1.0/1.8)
    return edges
"""
 
if __name__ == '__main__': 
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    video_file = args['video']
    params = args['params'] 
    cam = params['cam'][cam_num-1]
    video_reader = VideoReader(video_file)
    ldr_map = loadLDRCamMap(args['map'])

    #(tx,ty,tz) = (-0.50000000000000004, -0.2875, 0.34)
    (tx,ty,tz) = (-0.50000000000000004, 0.31, 0.34)
    (rx,ry,rz) = (0.044,0.0291,0.0115)
    C_current = array([tx,ty,tz,rx,ry,rz])
    BATCH_SIZE = 1 

    from multiprocessing import Pool
    pool = Pool(10)

    while True:
        
        batch_data = [ ]
        while len(batch_data) < BATCH_SIZE:
            (success, I, raw_pts) = getNextData(video_reader, ldr_map)
            if not success:
                break
            batch_data.append( [I.copy(), raw_pts] )

        batch_data = pool.map(processData, batch_data)
        #batch_data = processBatch(batch_data)
         
        count = 0
        while count < 30:
            count +=1


            out = gridsearch(C_current, batch_data, cam)
            print out[1]
            print out[0]
            if np.all(C_current == out[0]):
                break
            C_current = out[0]
            for idx in range(len(batch_data)):
                if idx != len(batch_data)-1:
                    continue
                proc_pts = batch_data[idx][3]
                (pix, pts_wrt_cam) = computeReprojection(C_current, proc_pts, cam)
                mask = computeMask(pix, pts_wrt_cam)
                px = pix[1,mask]
                py = pix[0,mask]
                E_show = batch_data[idx][2].copy()
                for p in range(4):
                    E_show[px+p,py] = 255
                    E_show[px,py+p] = 255
                    E_show[px-p,py] = 255
                    E_show[px,py-p] = 255
                imshow('viz', cv2.pyrDown(E_show/255.0))
                waitKey(5)
            
    
            #imshow('display', I)
            #key = chr((waitKey() & 255))
