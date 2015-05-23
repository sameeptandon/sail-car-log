import cv2
import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import NearestNeighbors
from rigid_transform_3D import rigid_transform_3D

def icp(a, b, initR=None, initt=None, num_iterations = 10):
    '''
    The Iterative Closest Point estimator.
    Takes two cloudpoints a[x,y], b[x,y] and the number of iterations.
    Returns the affine transform that transforms
    the cloudpoint a to the cloudpoint b.
    Note:
        (1) This method works for cloudpoints with minor
        transformations. Thus, the result depents greatly on
        the initial pose estimation.
        (2) A large number of iterations does not necessarily
        ensure convergence. Contrarily, most of the time it
        produces worse results.

    TODO(rchengyue): Still need to test.
    '''

    src = a.copy()
    dst = b.copy()

    # Initialize Rotation and translation matrices.
    if initR is None:
      R = np.mat(np.identity(3))
    else:
      R = np.mat(initR)
    if initt is None:
      t = np.mat(np.random.rand(3, 1))
    else:
      t = np.mat(initt)

    # Make R a proper rotation matrix, force orthonormal.
    U, S, Vt = np.linalg.svd(R)
    R = U*Vt
    # Transform src using rotation and translation matrix.
    src = R*src.T + np.tile(t, (1, len(src)))
    src = src.T

    for i in range(num_iterations):

        # Find the nearest neighbours between the current source and the
        # destination cloudpoint.
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(dst)
        distances, indices = nbrs.kneighbors(src)

        # Compute the transformation between the current source
        # and destination cloudpoint via nearest neighbors.
        R_new, t_new = rigid_transform_3D(src, np.mat(dst[indices.T][0]),initR=initR)

        # Transform the previous source and update the
        # current source cloudpoint.
        src = R_new*src.T + np.tile(t_new, (1, len(src)))
        src = src.T

        # Save the transformation from the actual source cloudpoint
        # to the destination.
        R = np.dot(R, R_new)
        t = t + t_new
    return R, t
