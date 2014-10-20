import numpy as np
import glob
import sys
from ArgParser import parse_args
from GPSTransforms import IMUTransforms
from LidarTransforms import utc_from_gps_log_all
from GPSReader import GPSReader
from scipy.spatial import cKDTree
from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt
from LaneMarkingHelper import get_transforms, DataTree

class LaneClassifier(object):
    def __init__(self, args, multilane_npz):
        self.args = args

        lanes_npz = np.load(multilane_npz)
        self.num_lanes = lanes_npz['num_lanes']

        self.lanes = [None for x in range(self.num_lanes)]

        for i in xrange(self.num_lanes):
            self.lanes[i] = DataTree(lanes_npz['lane' + str(i)])

        self.imu, _, _ = get_transforms(args)
        self.pos = self.imu[:, :3, 3]

    def classifyNearbyLanes(self, pos_idx):
        closest_lanes = np.ones((self.num_lanes, 2)) * -1
        for i in xrange(self.num_lanes):
            (d, idx) = self.lanes[i].tree.query(self.pos[pos_idx, :])
            closest_lanes[i] = np.array((d, idx))

        potential_lanes = np.argsort(closest_lanes[:, 0], axis=0)

        # pts_idx = self.ground_tree.query_ball_point(self.pos[pos_idx, :], radius)
        # pts = self.ground[pts_idx, :]

        # # Remove points that are outside the lanes
        xform = self.imu[pos_idx]
        inv_xform = np.linalg.inv(xform)

        closest_pt_left = None
        closest_pt_right = None

        for lane_num in potential_lanes:
            _, idx = self.lanes[lane_num].tree.query(self.pos[pos_idx], k=1)
            hom_pt = np.hstack((self.lanes[lane_num].pts[idx, :], 1))
            closest_pt_wrt_car = np.dot(hom_pt, inv_xform.T)
            norm = np.linalg.norm(closest_pt_wrt_car)

            if closest_pt_wrt_car[1] >= 0:
                if closest_pt_left == None or norm < closest_pt_left[0]:
                    closest_pt_left = (norm, closest_pt_wrt_car, lane_num)
            if closest_pt_wrt_car[1] < 0:
                if closest_pt_right == None or norm < closest_pt_right[0]:
                    closest_pt_right = (norm, closest_pt_wrt_car, lane_num)

        print pos_idx, closest_pt_left[-1], closest_pt_right[-1]
        # hom_pts = np.hstack((pts, np.ones((pts.shape[0], 1))))
        # rot_pts = np.dot(hom_pts, inv_xform.T)

        # pt_mask = (rot_pts[:, 1] < lane_boundary[0, 1])  & \
        #           (rot_pts[:, 1] > lane_boundary[1, 1])
        # filt_pts = np.dot(rot_pts[pt_mask], xform.T)

        # pts = filt_pts[:, :3]

        # if pts.shape[0] < 10:
        #     return (None,) * 3

        # # number of inliers, (idx0, idx1, idx2)
        # best_model = (0, np.array(0))

        # if end >= self.filt_ground.shape[0]:
        #     print 'resizing'
        #     self.filt_ground = np.concatenate((self.filt_ground,
        #                                        np.empty(self.filt_ground.shape)))
        #     print 'done'

        # self.filt_ground[start:end, :] = new_pts
        # self.filt_ground_idx = end

        # return n, p0, err

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    lanes_file = glob.glob(sys.argv[1] + '/*done.npz')[0]
    lc = LaneClassifier(args, lanes_file)

    for i in xrange(0, lc.pos.shape[0], 25):
        # print '%d/%d' % (i, lc.pos.shape[0])
        lc.classifyNearbyLanes(i)
