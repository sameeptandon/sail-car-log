import numpy as np
from numpy import sin, cos
from transformations import euler_matrix
"""
pixels is Nx2 matrix of coordinates,
cam is camera structure
"""
def pixelTo3d(pixels, cam):
  height = 1.105 # constants based on camera setup
  R_camera_pitch = euler_matrix(cam['rot_x'], cam['rot_y'], cam['rot_z'], 'sxyz')[0:3,0:3]
  KK = cam['KK']
  N = pixels.shape[0]
  assert(pixels.shape[1] == 2)
  # compute X/Z and Y/Z using inv(KK*R)*pixels
  Pos_S = np.linalg.solve(np.dot(KK,R_camera_pitch), np.concatenate((pixels,np.ones([N,1])), axis=1).transpose()).transpose()
  Y = np.ones((N))*height
  Z = Y/Pos_S[:,1]
  X = Pos_S[:,0]*Z
  return np.vstack((X,Y,Z)).transpose()
