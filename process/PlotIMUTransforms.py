import sys
import pylab
from ArgParser import *
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from VtkRenderer import *

if __name__ == '__main__': 
    args = parse_args(sys.argv[1], sys.argv[2])

    gps_reader = GPSReader(args['gps'])
    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)

    T = imu_transforms.shape[0]
    t = pylab.arange(0, T-1)
    pylab.plot(t, imu_transforms[t+1,2,3]-imu_transforms[t,2,3])
    #pylab.plot(t, imu_transforms[t+1,0,3]-imu_transforms[t,0,3])
    #pylab.plot(t, imu_transforms[t+1,1,3]-imu_transforms[t,1,3])
    #pylab.plot(t, imu_transforms[t,2,3])
    pylab.show()


    print imu_transforms[:,0:3,3]
