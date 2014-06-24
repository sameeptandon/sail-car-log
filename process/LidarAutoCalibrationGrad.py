import sys
import cv2
import scipy.weave
import itertools
import numpy as np
from ArgParser import parse_args
from scipy.signal import convolve2d
from scipy.optimize import fmin_l_bfgs_b
import matplotlib.pyplot as plt
from fabric.colors import green, red
from VideoReader import VideoReader
from LidarTransforms import loadLDRCamMap, loadLDR, R_to_c_from_l_old
from ColorMap import heatColorMapFast
from transformations import euler_matrix

# cam 5
#(tx,ty,tz) = (-0.3, 1.4, 0.55)
#(rx,ry,rz) = (0.0,-1*np.pi/2+0.1,0.0)

# cam 4
#(tx,ty,tz) = (0.5, -0.75, 0.35)
#(rx,ry,rz) = (0.0,np.pi/2,0.0)

# cam 3
(tx, ty, tz) = (-0.7, -0.2, 0.5)
(rx, ry, rz) = (0.0, 0.05, 0.0)

#(rx,ry,rz) = (0.0,0.0,-0.05)
C_global = np.array([tx, ty, tz, rx, ry, rz])

IMG_WIDTH = 2080
IMG_HEIGHT = 1040

t_eps = 0.1
r_eps = 2 * np.pi / 180

# Things to tweak
# - Bounds
# - Step size (1e-6 too small, 1e-5 seems to work)
# - L1 vs L2
# - use scans instead of sweep?
# - blurriness of the image
# - intrinsics / distortion
# - Look at distribution of parameters


def computeDistanceTransform(D, gamma, alpha):
    logD = np.log(D)
    logD = logD.astype(np.float32)
    logD = computeLogDistanceTransform(logD, gamma)
    F = np.exp(logD)

    return alpha * D + (1 - alpha) * F

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
    lg = np.log(gamma)
    code = \
    '''
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
    '''
    scipy.weave.inline(code, ['D', 'width', 'height', 'lg'], headers=['<algorithm>'],
                       type_converters=scipy.weave.converters.blitz)

    return D


def generateEdgeFilterKernels():
    kernels = []
    for x in range(3):
        for y in range(3):
            K = np.zeros((3, 3))
            K[1, 1] = 1.0
            K[x, y] = -1.0
            if (x != 1 and y != 1):
                kernels.append(K)
    return kernels

def processPointCloud(raw_pts):
    # add rotational angle and distance to pts
    pts = np.zeros((raw_pts.shape[0], raw_pts.shape[1] + 2), dtype=np.float32)
    pts[:, :-2] = raw_pts
    pts[:, -2] = np.arctan2(pts[:, 1], pts[:, 0]) + np.pi
    pts[:, -1] = np.sqrt(np.sum(pts[:, 0:3] ** 2, axis=1))

    pts = pts[pts[:, -2].argsort()]  # sort on rotational angle
    pts = pts[pts[:, 4].argsort(kind='mergesort')]  # stable sort on laser num

    pts[0, 3] = 0.0
    pts[-1, 3] = 0.0
    """
    pts[1:-1,3] = np.maximum(pts[0:-2,-1] - pts[1:-1, -1],
                             pts[2:, -1] - pts[1:-1, -1])
    pts[1:-1,3] = np.maximum(pts[1:-1,3], 0)

    """
    for idx in range(1, pts.shape[0] - 1):
        if pts[idx, 4] == pts[idx - 1, 4] and pts[idx, 4] == pts[idx + 1, 4]:
            pts[idx, 3] = max(pts[idx - 1, -1] - pts[idx, -1],
                              pts[idx + 1, -1] - pts[idx, -1],
                              0)
        else:
            pts[idx, 3] = 0.0
    #pts = pts[pts[:,0] > 0, :]
    pts = pts[pts[:, 3] > 1.0]

    # Only consider points which are in the image
    (pix, pts_wrt_cam) = computeReprojection(C_global, pts, cam)
    mask = computeMask(pix, pts_wrt_cam)
    pts = pts[mask, :]

    return pts


def computeReprojection(C, raw_pts, cam):
    pts = raw_pts[:, 0:3].copy()
    pts[:, 0] += C[0]
    pts[:, 1] += C[1]
    pts[:, 2] += C[2]
    R = euler_matrix(C[3], C[4], C[5])[0:3, 0:3]
    pts_wrt_cam = np.dot(R, np.dot(R_to_c_from_l_old(cam), pts.transpose()))
    pix = np.around(np.dot(cam['KK'],
        np.divide(pts_wrt_cam[0:3, :], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    return (pix, pts_wrt_cam)


def computeMask(pix, pts_wrt_cam):
    width = 8
    mask = np.logical_and(True, pix[0, :] > 0 + width / 2)
    mask = np.logical_and(mask, pix[1, :] > 0 + width / 2)
    mask = np.logical_and(mask, pix[0, :] < IMG_WIDTH - width / 2)
    mask = np.logical_and(mask, pix[1, :] < IMG_HEIGHT - width / 2)
    mask = np.logical_and(mask, pts_wrt_cam[2, :] > 0)
    return mask


def computeReprojectionScore(C, pts, I, cam):
    (pix, pts_wrt_cam) = computeReprojection(C, pts, cam)
    px = pix[1, :]
    py = pix[0, :]
    score = np.sum(bilinear_interpolate(I, px, py))
    score = score / px.size
    return score


def gridsearch(C, batch, cam):
    m = range(-1, 2)
    step_t = 0.01
    step_r = 0.003

    best_score = -float("inf")
    scores = np.zeros((3 ** 3, 1))
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

    #print scores
    #print current_score
    if np.sum(scores > current_score) >= 3 ** 3 / 2:
        return (best_C, best_score)
    else:
        return (C, current_score)


def bilinear_interpolate(dist_transform, x, y):
    dt = dist_transform
    M = dt.shape[0]
    N = dt.shape[1]

    reg = 1e-10  # PARAM
    x[x % 1 == 0] += reg
    y[y % 1 == 0] += reg

    x1 = np.array(np.floor(x), dtype=np.int32)
    x2 = x1 + 1
    y1 = np.array(np.floor(y), dtype=np.int32)
    y2 = y1 + 1

    d = np.zeros(x.size)
    in_bounds = np.logical_and(
        np.logical_and(x1 >= 0, y1 >= 0), np.logical_and(x2 < N, y2 < M))
    out_bounds = np.logical_not(in_bounds)
    # FIXME May want to penalize more heavily
    d[out_bounds] = np.max(dist_transform) + np.sqrt(x[out_bounds] ** 2 + y[out_bounds] ** 2)

    x = x[in_bounds]
    x1 = x1[in_bounds]
    x2 = x2[in_bounds]
    y = y[in_bounds]
    y1 = y1[in_bounds]
    y2 = y2[in_bounds]

    # Interpolate in x-direction for y1 and y2
    d_x1 = (x2 - x) / (x2 - x1 + reg) * \
        dt[y1, x1] + (x - x1) / (x2 - x1 + reg) * dt[y1, x2]
    d_x2 = (x2 - x) / (x2 - x1 + reg) * \
        dt[y2, x1] + (x - x1) / (x2 - x1 + reg) * dt[y2, x2]
    # Interpolate in y-direction
    d[in_bounds] = (y2 - y) / (y2 - y1 + reg) * d_x1 + \
        (y - y1) / (y2 - y1 + reg) * d_x2

    return d


class CalibScore(object):

    def __init__(self, cam, batch):
        self.batch = batch
        self.cam = cam

    def score(self, C):
        score = 0.0
        for p in self.batch:
            E = p[2]
            proc_pts = p[3]
            score = score + computeReprojectionScore(C, proc_pts, E, self.cam)
        print score
        return score


def optimize_calib(C_init, batch, cam):
    scorer = CalibScore(cam, batch)
    eps = np.array([t_eps, t_eps, t_eps, r_eps, r_eps, r_eps])
    C_min = C_init - eps
    C_max = C_init + eps
    [x, f, d] = fmin_l_bfgs_b(
        scorer.score, C_init, approx_grad=True, bounds=zip(C_min, C_max), epsilon=1e-5)
    print 'success:', d['warnflag'] == 0
    return (x, f)


def drawReprojection(C, pts, I, cam):
    (pix, pts_wrt_cam) = computeReprojection(C, pts, cam)
    mask = computeMask(pix, pts_wrt_cam)

    px = pix[1, mask]
    py = pix[0, mask]
    intensity = pts[mask, 3]
    colors = heatColorMapFast(intensity, 0, 100)
    I[px, py, :] = colors[0, :, :]

    cv2.imshow('display', I)
    cv2.waitKey(1000)


def getNextData(reader, LDRFrameMap):
    for idx in range(10):
        (success, img) = reader.getNextFrame()
        #img = cv2.flip(img,-1)
        if not success:
            reader.setFrame(3)
    ldr_frame = loadLDR(LDRFrameMap[reader.framenum])
    return (success, img, ldr_frame)


def processData(data):
    I, pts = data
    E = processImage(I)
    proc_pts = processPointCloud(pts)
    dist = np.sqrt(np.sum(proc_pts[:, 0:3] ** 2, axis=1))
    proc_pts = proc_pts[dist > 3, :]
    #proc_pts = proc_pts[ proc_pts[:, 3] > 2.0, :]
    return [I, pts, E, proc_pts]


def processBatch(batch):
    processed = []
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
    kernels = generateEdgeFilterKernels()

    # convert the image to grayscale
    E = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)

    # run an edge filter
    edges = cv2.filter2D(E, cv2.CV_8U, np.zeros((1, 1)))

    for k in kernels:
        edges = np.maximum(edges, np.abs(cv2.filter2D(E, cv2.CV_8U, k)))

    edges = computeDistanceTransform(edges + 1, 0.98, 1.0 / 1.8)
    return edges


def processImage2(I):
    # convert the image to grayscale
    E = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    E = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    G_x, G_y = dgauss_filt(1.0)
    I_x = -convolve2d(E, G_x, mode='same')
    I_y = -convolve2d(E, G_y, mode='same')
    I_mag = np.sqrt(I_x ** 2 + I_y ** 2)
    I_mag = np.array((I_mag < 10.0) * 255, dtype=np.uint8)
    edges = computeDistanceTransform(I_mag, 0.98, 1.0 / 2.0)
    #edges = cv2.distanceTransform(I_mag, cv.CV_DIST_L1, cv.CV_DIST_MASK_5)
    #print edges
    #cv2.imshow("test", edges)
    #cv2.waitKey(0)
    #print edges.shape
    #import matplotlib.pyplot as plt
    #plt.imshow(edges[0])
    #plt.show()
    #return edges[0]
    return edges


if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    video_file = args['video']
    params = args['params']
    cam = params['cam'][cam_num - 1]
    video_reader = VideoReader(video_file)
    ldr_map = loadLDRCamMap(args['map'])

    #(tx,ty,tz) = (-0.50000000000000004, -0.2875, 0.34)
    C_current = np.array([tx, ty, tz, rx, ry, rz])
    BATCH_SIZE = 1

    from multiprocessing import Pool
    pool = Pool(4)

    while True:
        batch_data = []
        while len(batch_data) < BATCH_SIZE:
            (success, I, raw_pts) = getNextData(video_reader, ldr_map)
            if not success:
                break
            batch_data.append([I.copy(), raw_pts])

        batch_data = pool.map(processData, batch_data)
        #batch_data = [processData(x) for x in batch_data]

        count = 0
        while count < 1:
            count += 1

            out_grid = gridsearch(C_current, batch_data, cam)
            out_grad = optimize_calib(C_global, batch_data, cam)

            print 'obj gridsearch:', green(out_grid[1])
            print 'x_opt', out_grid[0]
            print 'obj l-bfgs-b:', green(out_grad[1])
            print 'x_opt', out_grad[0]
            print 'diff', red(out_grid[1] - out_grad[1])

            out_opt = out_grad

            if np.all(C_current == out_opt[0]):
                print 'All same'
                break
            for idx in range(len(batch_data)):
                if idx != len(batch_data) - 1:
                    continue
                proc_pts = batch_data[idx][3]
                (pix, pts_wrt_cam) = computeReprojection(
                    out_opt[0], proc_pts, cam)
                mask = computeMask(pix, pts_wrt_cam)
                px = pix[1, mask]
                py = pix[0, mask]

                pts = batch_data[idx][1]

                drawReprojection(
                    out_opt[0], pts, batch_data[idx][0].copy(), cam)

                '''
                E_show = batch_data[idx][2].copy()
                for p in range(4):
                    E_show[px+p, py] = 255
                    E_show[px, py+p] = 255
                    E_show[px-p, py] = 255
                    E_show[px, py-p] = 255
                cv2.imshow('viz', cv2.pyrDown(E_show/255.0))
                cv2.waitKey(100)
                '''

            #cv2.imshow('display', I)
            #key = chr((cv2.waitKey() & 255))
