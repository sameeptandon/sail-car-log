from LidarTransforms import * 
import sys, os
from VideoReader import *
from CameraParams import * 
import cv2
from cv2 import imshow, waitKey
from numpy.linalg import norm
from ColorMap import *
from numpy import exp, log
from transformations import euler_matrix
import scipy.weave 

global counter
counter = 0

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
            K[1,1] = 3.0
            K[x,y] = -3.0
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
    for idx in range(1,pts.shape[0]-1):
        if pts[idx,4] == pts[idx-1,4] and pts[idx,4] == pts[idx+1,4]: 
            pts[idx,3] = max(pts[idx-1,-1] - pts[idx,-1], 
                             pts[idx+1,-1] - pts[idx,-1],
                             0)
        else:
            pts[idx,3] = 0.0
    return pts


def computeReprojection(C, raw_pts, cam):
    pts = raw_pts[:, 0:3].copy()
    pts[:, 0] += C[0]
    pts[:, 1] += C[1]
    pts[:, 2] += C[2]
    R = euler_matrix(C[3], C[4], C[5])[0:3,0:3]
    pts_wrt_cam = dot(R, dot(R_to_c_from_l(cam), pts.transpose()))
    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    return (pix, pts_wrt_cam)

def computeMask(pix, pts_wrt_cam):
    mask = np.logical_and(True, pix[0,:] > 0)
    mask = np.logical_and(mask, pix[1,:] > 0)
    mask = np.logical_and(mask, pix[0,:] < 1279)
    mask = np.logical_and(mask, pix[1,:] < 959)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    return mask

def computeReprojectionScore(C, pts, I, cam):
    global counter
    counter = counter + 1
    (pix, pts_wrt_cam) = computeReprojection(C, pts, cam)
    mask = computeMask(pix, pts_wrt_cam)

    px = pix[1,mask]
    py = pix[0,mask]
    return -np.sum(I[px,py])


def gridsearch(C, batch, cam):
    m = range(-1,2)
    step_t = 0.01
    step_r = 0.001

    best_score = float("inf")
    best_d = None

    for dtx in m:
        for dty in m:
            for dtz in m:
                for drx in m:
                    for dry in m:
                        for drz in m:
                            d = np.array([step_t*dtx, step_t*dty, step_t*dtz, step_r*drx, step_r*dry, step_r*drz])
                            C_new = C + d
                            score = 0
                            for p in batch:
                                E = p[2]
                                proc_pts = p[3]
                                score = score + computeReprojectionScore(C_new, proc_pts, E, cam)
                            if score < best_score:
                                best_score = score
                                best_d = d
    return (C+best_d, best_score)



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
            return None
    ldr_frame = loadLDR(LDRFrameMap[reader.framenum])
    #print LDRFrameMap[reader.framenum]
    return (success, img, ldr_frame)
  

def processData(data):
    I, pts = data
    E = processImage(I);
    proc_pts = processPointCloud(pts)
    proc_pts = proc_pts[ proc_pts[:, 3] > 0.5, :]
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

kernels = generateEdgeFilterKernels()
def processImage(I):
    # convert the image to grayscale
    E = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)

    # run an edge filter
    edges = cv2.filter2D(E, cv2.CV_8U, np.zeros((1,1)))
    
    for k in kernels: 
        edges = np.maximum(edges, np.abs(cv2.filter2D(E, cv2.CV_8U, k)))

    edges = computeDistanceTransform(edges+1, 0.95, 1.0/3)
    return edges
 
if __name__ == '__main__': 
    video_filename = sys.argv[1] 
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    cam = getCameraParams()[cam_num - 1] 
    video_reader = VideoReader(video_filename)
    ldr_frame_map = loadLDRCamMap(sys.argv[2])
    (tx,ty,tz,rx,ry,rz) = (-0.30000000000000004, 0.5, 0.4299999999999998, -0.09800000000000006, 0.019999999999999987, 0.010999999999999982)
    (tx,ty,tz,rx,ry,rz) = (-0.35493086,  0.49796525,  0.43775339, -0.0986506, 0.01898486,  0.01463721)
    C_current = array([tx,ty,tz,rx,ry,rz])
    BATCH_SIZE = 50

    from multiprocessing import Pool
    pool = Pool(8)

    while True:
        
        batch_data = [ ]
        while len(batch_data) < BATCH_SIZE:
            (success, I, raw_pts) = getNextData(video_reader, ldr_frame_map)
            if not success:
                break
            batch_data.append( [I, raw_pts] )

        batch_data = pool.map(processData, batch_data)
        #batch_data = processBatch(batch_data)
       
        count = 0
        while count < 10:
            count +=1


            #cv2.imshow('edges', edges/255.0)
            """
            from scipy.optimize import brute
    
            step_size = 0.0001
            step_sizes = [0.0005, 0.0005, 0.0005, 0.0001, 0.0001, 0.0001]
            ranges = [(x-y, x+y) for x,y in zip(C_current, step_sizes)]
            print ranges
            out = brute(computeReprojectionScore, ranges, args=(pts, edges, cam), full_output=True, Ns=1)
            """
            out = gridsearch(C_current, batch_data, cam)
            #print out[0]
            #print out[1]
            #print counter
            #drawReprojection(out[0], pts, E, cam)
            E = batch_data[0][2]
            proc_pts = batch_data[0][3]

            (pix, pts_wrt_cam) = computeReprojection(C_current, proc_pts, cam)
            mask = computeMask(pix, pts_wrt_cam)
            px = pix[1,mask]
            py = pix[0,mask]
            edges = E.copy()
            edges[px,py] = 255
            imshow('score', edges/255.0)
            waitKey(10)
            C_current = out[0]
            print C_current
    
            #imshow('display', I)
            #key = chr((waitKey() & 255))
