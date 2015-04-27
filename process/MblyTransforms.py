import numpy as np
import os, sys
import glob
import mbly_obj_pb2
import cv2
from LidarTransforms import R_to_c_from_l

def calibrateMblyPts(pts, T, R):
    """ Transforms the mobileye output into the lidar's FoR.
        TODO: Add save these values to params
    """
    # R = np.eye(3)
    # pts[:, 0] += 0.762
    # pts[:, 1] += 0.0381
    # pts[:, 2] += -0.9252
    pts[:, :3] += np.tile(T, (pts.shape[0], 1))
    pts_wrt_lidar = np.dot(R, pts[:,:3].T).T
    return pts_wrt_lidar

def projectPoints(mbly_data, args, T, R):
    """ Projects mobileye points into the camera's frame
        Args: mbly_data, the output from loadMblyWindow
              args, the output from parse_args
    """
    params = args['params']
    cam_num = args['cam_num']
    cam = params['cam'][cam_num]

    # Move points to the lidar FoR
    pts_wrt_lidar = calibrateMblyPts(mbly_data, T, R)

    # Move the points to the cam FoR
    pts_wrt_cam = pts_wrt_lidar +\
      cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_cam = np.dot(R_to_c_from_l(cam), pts_wrt_cam.transpose())

    # Project the points into the camera space
    (pix, J) = cv2.projectPoints(pts_wrt_cam.transpose(),
        np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]),
        cam['KK'], cam['distort'])
    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)

    mbly_data_projected = np.hstack((mbly_data, pix.transpose()))
    return mbly_data_projected

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
