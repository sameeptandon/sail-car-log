import numpy as np
import sys
from ArgParser import parse_args
from GPSTransforms import IMUTransforms
from LidarTransforms import utc_from_gps_log_all
from GPSReader import GPSReader
from scipy.spatial import cKDTree
from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt

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
        self.backup_lanes = [None for x in range(self.num_lanes)]

        for i in xrange(self.num_lanes):
            self.lanes[i] = np.array(lanes_npz['lane' + str(i)])
            self.lanes_tree[i] = cKDTree(self.lanes[i])
            self.backup_lanes[i] = self.lanes[i].copy()

        self.imu, _, _ = get_transforms(args)
        self.pos = self.imu[:, :3, 3]

        self.ground = np.load(ground_npz)['data'][:, :3]
        self.ground_tree = cKDTree(self.ground)

        self.planes = None
        self.filt_ground = np.empty(self.ground.shape)
        self.filt_ground_idx = 0

    def fitPlane(self, pts):
        p0 = np.mean(pts, axis=0)
        A = pts - p0
        u, _, _ = np.linalg.svd(A.T, full_matrices=False)
        n = u[:, 2]
        err = np.sum(np.dot(A, n)**2)**0.5 / A.shape[0]
        return n, p0, err

    def planeFromPoints(self, pts):
        p0 = pts[0, :]
        u = pts[1, :] - p0
        v = pts[2, :] - p0
        n = np.cross(u, v)
        n /= np.linalg.norm(n)
        if np.linalg.norm(n) == 0:
            print n
        return n, p0

    def getInliers(self, n, p0, pts, threshold):
        pts_ = pts - np.tile(p0, (pts.shape[0], 1))
        dist = np.dot(pts_, n)
        inliers = np.abs(dist) < threshold
        return inliers

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
            self.planes = np.hstack((n, p0))[np.newaxis, :]
        else:
            # Use a moving average to smooth out planes
            n = self.planes[-1, :3] * 0.3 + n * 0.7
            self.planes = np.vstack((self.planes, np.hstack((n, p0))))


        new_pts = pts[inliers][::32]
        start = self.filt_ground_idx
        end = start + new_pts.shape[0]

        if end >= self.filt_ground.shape[0]:
            print 'resizing'
            self.filt_ground = np.concatenate((self.filt_ground,
                                               np.empty(self.filt_ground.shape)))
            print 'done'

        self.filt_ground[start:end, :] = new_pts
        self.filt_ground_idx = end

        return n, p0, err

    def correctLanes(self):
        plane_tree = cKDTree(self.planes[:, 3:])
        for i in xrange(self.num_lanes):
            lane = self.lanes[i]

            _, closest_idx = plane_tree.query(lane)

            planes = np.array([self.planes[idx] for idx in closest_idx])
            p0 = planes[:, 3:]
            n = planes[:, :3]

            v = lane - p0
            dist = np.sum(v * n, axis=1)

            lane -= n * dist[:, np.newaxis]
            t = np.arange(0, lane.shape[0])

            xinter = UnivariateSpline(t, lane[:, 0], s=0)
            yinter = UnivariateSpline(t, lane[:, 1], s=0)
            zinter = UnivariateSpline(t, lane[:, 2], s=10)

            self.lanes[i] = np.column_stack((xinter(t), yinter(t), zinter(t)))

            # if i == 0:
            #     plt.plot(t, self.lanes[i][:,2], 'r.', t, lane[:,2], 'g-')
            #     plt.show()


    def exportData(self, file_name):
        lanes = {}
        lanes['num_lanes'] = self.num_lanes
        lanes['planes'] = self.planes
        lanes['filt_ground'] = self.filt_ground[:self.filt_ground_idx]
        for num in xrange(self.num_lanes):
            lane = self.lanes[num]
            lanes['lane' + str(num)] = lane

        np.savez(file_name, **lanes)


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
