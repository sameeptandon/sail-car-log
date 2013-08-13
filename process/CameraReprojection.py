import numpy as np
from numpy import sin, cos

"""
pixels is Nx2 matrix of coordinates, 
cam is camera structure
"""
def pixelTo3d(pixels, cam):
  pitch = -cam['rot_x'] # constants based on camera setup
  height = 1.105 # constants based on camera setup 
  N = pixels.shape[0]
  assert(pixels.shape[1] == 2)
  Z = ((pixels[:,1]-cam['cv'])*sin(pitch)*height+cam['fy']*cos(pitch)*height)/(cos(pitch)*(pixels[:,1]-cam['cv'])-cam['fy']*sin(pitch))
  X = (cos(pitch)*Z-sin(pitch)*height)*(pixels[:,0]-cam['cu'])/cam['fx']
  Y = np.zeros((N))
  return np.vstack((X,Y,Z)).transpose()
