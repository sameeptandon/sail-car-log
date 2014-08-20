import numpy as np
import sys
from ArgParser import parse_args
from GPSTransforms import IMUTransforms
from LidarTransforms import utc_from_gps_log_all
from GPSReader import GPSReader
from scipy.spatial import cKDTree

def get_transforms(args, mark='mark1'):
    """ Gets the IMU transforms for a run """
    gps_reader = GPSReader(args['gps_' + mark])
    gps_data = gps_reader.getNumericData()
    gps_times = utc_from_gps_log_all(gps_data)
    imu_transforms = IMUTransforms(gps_data)
    return imu_transforms, gps_data, gps_times

class PlaneFitter(object):
    def __init__(self, args, ground_npz, multilane_npz):
        self.args = args

        lanes_npz = np.load(multilane_npz)
        self.num_lanes = lanes_npz['num_lanes']
        self.lanes = [None for x in range(self.num_lanes)]
        self.lanes_tree = [None for x in range(self.num_lanes)]

        for i in xrange(self.num_lanes):
            self.lanes[i] = np.array(lanes_npz['lane' + str(i)])
            self.lanes_tree[i] = cKDTree(self.lanes[i])

        self.imu, _, _ = get_transforms(args)
        self.pos = self.imu[:, :3, 3]

        self.ground = np.load(ground_npz)['data'][:, :3]
        self.ground_tree = cKDTree(self.ground)

        self.planes = None

    def fitPlane(self, pts):
        p0 = np.mean(pts, axis=0)
        A = pts - p0
        u, _, _ = np.linalg.svd(np.transpose(A), full_matrices=False)
        n = u[2]
        err = np.sum(np.dot(A, n)**2) / A.shape[0]
        return n, p0, err

    def planeFromPoints(self, pts):
        p0 = pts[0, :]
        u = pts[1, :] - p0
        v = pts[2, :] - p0
        n = np.cross(u, v)
        n /= np.linalg.norm(n)
        return n, p0

    def getInliers(self, n, p0, pts, threshold):
        pts_ = pts - np.tile(p0, (pts.shape[0], 1))
        dist = np.dot(pts_, n)
        inliers = np.abs(dist) < threshold
        return inliers

    def findGroundPlane(self, pos_idx, radius=100.0, threshold=0.1, n_iter=15):
        pts_idx = self.ground_tree.query_ball_point(self.pos[pos_idx, :], radius)
        pts = self.ground[pts_idx, :]
        # number of inliers, (idx0, idx1, idx2)
        best_model = (0, np.array(0))
        for i in xrange(n_iter):
            rand_idx = np.random.choice(np.arange(0, pts.shape[0]), 3)
            rand_pts = pts[rand_idx, :]

            n, p0 = self.planeFromPoints(rand_pts)
            inliers = self.getInliers(n, p0, pts, threshold)
            num_inliers = np.sum(inliers)

            if num_inliers > best_model[0]:
                best_model = (num_inliers, rand_idx)

        n, p0 = self.planeFromPoints(pts[best_model[1], :])
        inliers = self.getInliers(n, p0, pts, threshold)

        n, p0, err = self.fitPlane(pts[inliers, :])

        if self.planes == None:
            self.planes = np.hstack((n, p0))
        else:
            self.planes = np.vstack((self.planes, np.hstack((n, p0))))

        return n, p0, err

    def correctLanes(self, pos_idx, n, p0, radius=100.0):
        for i in xrange(self.num_lanes):
            lane = self.lanes[i]
            tree = self.lanes_tree[i]
            _, i = lane_pts = tree.query(self.pos[pos_idx, :], k=1)
            lane_idx = tree.query_ball_point(lane[i], radius)
            part_lane = lane[lane_idx]

            v = part_lane - p0
            dist = np.dot(v, n)
            part_lane[:, 2] -= (n * dist[:, np.newaxis])[:, 2]

    def exportData(self, file_name):
        lanes = {}
        lanes['num_lanes'] = self.num_lanes
        lanes['planes'] = self.planes
        for num in xrange(self.num_lanes):
            lane = self.lanes[num]
            lanes['lane' + str(num)] = lane

        np.savez(file_name, **lanes)


if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    plane_file = args['fullname'] + '_ground.npz'
    lanes_file = sys.argv[1] + '/multilane_points.npz'
    pf = PlaneFitter(args, plane_file, lanes_file)
    for i in xrange(0, pf.pos.shape[0], 500):
        print '%d/%d' % (i, pf.pos.shape[0])

        n, p0, err = pf.findGroundPlane(i, radius=200)
        pf.correctLanes(i, n, p0, radius=200)
        pf.exportData(sys.argv[1] + '/multilane_points_planar.npz')
