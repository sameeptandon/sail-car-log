import cv, cv2
import numpy as np


def warpPoints(P, pts):
    """
    warpPoints takes a list of points and performs a matrix transform on them

    P is a perspective transform matrix (3x3)
    pts is a 2xN matrix of (x, y) coordinates to be transformed
    """
    pts = np.vstack([pts, np.ones((1, pts.shape[1]))])
    out = np.dot(P, pts)
    return out[0:2] / out[2]
