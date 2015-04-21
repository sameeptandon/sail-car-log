import numpy as np
import os, sys
import glob
import mbly_obj_pb2

# def calibrateRadarPts(pts, params):
#     R = params['R_from_r_to_l']

#     pts[:, 0] += params['T_from_r_to_l'][0]
#     pts[:, 1] += params['T_from_r_to_l'][1]
#     pts[:, 2] += params['T_from_r_to_l'][2]

#     return np.dot(R, pts.transpose()).transpose()

class MblyLoader(object):
    def __init__(self, mbly_proto):
        pb_objs = mbly_obj_pb2.Objects()
        with open(mbly_proto) as f:
            pb_objs.ParseFromString(f.read())

        times = set([o.timestamp for o in pb_objs.object])
        self.times = np.array(sorted(times))

        self.objects = {}
        for o in pb_objs.object:
            if o.timestamp not in self.objects:
                self.objects[o.timestamp] = [o]
            else:
                self.objects[o.timestamp].append(o)
        # print self.objects

    def loadMblyWindow(self, microsec_since_epoch):
        """
        Loads the output of the MobilEye sensor closest to
        microseconds_since_epoch
        """
        idx = np.searchsorted(self.times, microsec_since_epoch * 1000L,
                              side='right')
        objs = self.objects[self.times[idx]]
        return objs

if __name__ == "__main__":
    loader = MblyLoader(sys.argv[1])
    loader.loadMblyWindow(loader.times[1] / 1000L)
    loader.loadMblyWindow(loader.times[1] / 1000L - 100000)
    loader.loadMblyWindow(loader.times[1] / 1000L + 300000)
