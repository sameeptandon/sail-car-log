import numpy as np
import sys
from ArgParser import parse_args
from GPSTransforms import IMUTransforms
from LidarTransforms import utc_from_gps_log_all
from GPSReader import GPSReader
from scipy.spatial import cKDTree
from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt
from LaneMarkingHelper import get_transforms, DataTree

class PlaneFitter(object):
    def __init__(self, args, ground_npz, multilane_npz):
        self.args = args

        lanes_npz = np.load(multilane_npz)
        self.num_lanes = lanes_npz['num_lanes']
        self.lanes = [None for x in range(self.num_lanes)]

        for i in xrange(self.num_lanes):
            self.lanes[i] = DataTree(lanes_npz['lane' + str(i)])

        self.imu, _, _ = get_transforms(args)
        self.pos = self.imu[:, :3, 3]

    def findGroundPlane(self, pos_idx, radius=100.0, threshold=0.05, n_iter=10):
        pts_idx = self.ground_tree.query_ball_point(self.pos[pos_idx, :], radius)
        pts = self.ground[pts_idx, :]

        # Remove points that are outside the lanes
        xform = self.imu[pos_idx]
        inv_xform = np.linalg.inv(xform)

        _, left_idx = self.lanes_tree[0].query(self.pos[pos_idx], k=1)
        _, right_idx = self.lanes_tree[-1].query(self.pos[pos_idx], k=1)

        lane_boundary = np.vstack((self.lanes[0][left_idx], self.lanes[-1][right_idx]))
        lane_boundary = np.hstack((lane_boundary, np.ones((2, 1))))
        lane_boundary = np.dot(lane_boundary, inv_xform.T)

        hom_pts = np.hstack((pts, np.ones((pts.shape[0], 1))))
        rot_pts = np.dot(hom_pts, inv_xform.T)

        pt_mask = (rot_pts[:, 1] < lane_boundary[0, 1])  & \
                  (rot_pts[:, 1] > lane_boundary[1, 1])
        filt_pts = np.dot(rot_pts[pt_mask], xform.T)

        pts = filt_pts[:, :3]

        if pts.shape[0] < 10:
            return (None,) * 3

        # number of inliers, (idx0, idx1, idx2)
        best_model = (0, np.array(0))

        if end >= self.filt_ground.shape[0]:
            print 'resizing'
            self.filt_ground = np.concatenate((self.filt_ground,
                                               np.empty(self.filt_ground.shape)))
            print 'done'

        self.filt_ground[start:end, :] = new_pts
        self.filt_ground_idx = end

        return n, p0, err

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    plane_file = args['fullname'] + '_ground.npz'
    lanes_file = sys.argv[1] + '/multilane_points.npz'
    pf = PlaneFitter(args, plane_file, lanes_file)

    for i in xrange(0, pf.pos.shape[0], 25):
    # for i in xrange(1000, 2000, 100):
        if i % 1000 == 0:
            print '%d/%d' % (i, pf.pos.shape[0])

        n, p0, err = pf.findGroundPlane(i, radius=25)

    pf.correctLanes()
    pf.exportData(sys.argv[1] + '/multilane_points_planar.npz')
