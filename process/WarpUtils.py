import cv, cv2
import numpy as np


def warpPoints(P, pts):
    pts = np.vstack([pts, np.ones((1, pts.shape[1]))])
    out = np.dot(P, pts)
    return out[0:2] / out[2]
