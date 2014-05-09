import sys
import matplotlib.pyplot as plt
import numpy as np
from ArgParser import *
from GPSReader import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from VtkRenderer import *

if __name__ == '__main__': 
    args = parse_args(sys.argv[1], sys.argv[2])

    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)

    T = imu_transforms.shape[0]
    t = np.arange(0, T-1)
    delta_z =  imu_transforms[t+1,2,3]-imu_transforms[t,2,3]

    plt.plot(t, delta_z)
    #plt.plot(np.arange(0, T), imu_transforms[:, 2, 3])
    #plt.plot(t, imu_transforms[t+1,0,3]-imu_transforms[t,0,3])
    #plt.plot(t, imu_transforms[t+1,1,3]-imu_transforms[t,1,3])
    #plt.plot(t, imu_transforms[t,2,3])
    plt.show()

    print imu_transforms[:,0:3,3]
