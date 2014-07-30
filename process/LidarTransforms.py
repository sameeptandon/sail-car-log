import os
import glob
import numpy as np
import bisect
from transformations import quaternion_from_matrix, quaternion_slerp,\
        quaternion_matrix

# PARAM
SWEEP_TIME_MICROSEC = 100000


def loadLDR(ldrfile):
    z = np.fromfile(ldrfile, dtype=np.float32)
    z = z.reshape((z.shape[0] / 6, 6))
    return z

def loadLDRCamMap(frame_cloud_map):
    map_file = open(frame_cloud_map, 'r')
    clouds = []
    frame_folder, map_name = os.path.split(frame_cloud_map)
    frame_folder = frame_folder + '/' + map_name.split('.')[0] + '_frames'

    for line in map_file:
        ldr_file = line.rstrip().split(' ')[1]
        clouds.append(frame_folder + '/' + ldr_file)
    map_file.close()

    return clouds

def utc_from_gps_log(log):
    return utc_from_gps(log[10], log[0])


def utc_from_gps_log_all(log):
    return utc_from_gps(log[:, 10], log[:, 0])


def utc_from_gps(gps_week, seconds, leap_seconds=16):
    """ Converts from gps week time to UTC time. UTC time starts from JAN 1,
        1970 and GPS time starts from JAN 6, 1980.

        http://leapsecond.com/java/gpsclock.htm
    """

    secs_in_week = 604800
    secs_gps_to_utc = 315964800

    utc = (gps_week * secs_in_week + seconds + secs_gps_to_utc -
        leap_seconds) * 1000000
    if type(gps_week) is np.ndarray:
        return np.array(utc, dtype=np.int64)
    else:
        return long(utc)


class LDRLoader(object):

    def __init__(self, ldr_dir):
        ldr_files = sorted(list(glob.glob(os.path.join(ldr_dir, '*.ldr'))))
        file_times = np.array([int(os.path.splitext(os.path.basename(f))[0]) for f in ldr_files], dtype=np.int64)
        sweep_end_times = file_times
        sweep_start_times = sweep_end_times - SWEEP_TIME_MICROSEC
        self.ldr_files = np.array(ldr_files)
        self.sweep_start_times = sweep_start_times
        self.sweep_end_times = sweep_end_times

    def loadLDRWindow(self, microsec_since_epoch, time_window_sec):
        time_window = time_window_sec * 1e6

        # Get files that may have points within the time window
        mask = (self.sweep_start_times >= microsec_since_epoch - time_window / 2.0 - SWEEP_TIME_MICROSEC) &\
               (self.sweep_end_times <= microsec_since_epoch + time_window / 2.0 + SWEEP_TIME_MICROSEC)
        ldr_files = self.ldr_files[mask].tolist()
        #print ldr_files
        sweep_end_times = self.sweep_end_times[mask].tolist()

        # Load the points from those times
        all_data = None
        all_times = None
        for (ldr_file, sweep_end_time) in zip(ldr_files, sweep_end_times):
            data = loadLDR(ldr_file)
            times = -1 * np.array(data[:, 5], dtype=np.int64) + sweep_end_time

            if all_data is None:
                all_data = data
                all_times = times
            else:
                all_data = np.vstack((all_data, data))
                all_times = np.concatenate((all_times, times))

        # Filter points within time window

        mask = (all_times >= microsec_since_epoch - time_window / 2.0) &\
               (all_times <= microsec_since_epoch + time_window / 2.0)
        if all_data is None:
            return None, None
        return all_data[mask, :], all_times[mask]


def R_to_c_from_l_old(cam):
    # hard coded calibration parameters for now
    R_to_c_from_l = np.array([[0.0, -1.0, 0.0],
                              [0.0, 0.0, -1.0],
                              [1.0, 0.0, 0.0]])

    return R_to_c_from_l

def R_to_c_from_l(cam):
    # hard coded calibration parameters for now
    R_to_c_from_l = np.array([[0.0, -1.0, 0.0],
                              [0.0, 0.0, -1.0],
                              [1.0, 0.0, 0.0]])
    R_to_c_from_l = np.dot(cam['R_to_c_from_l_in_camera_frame'], R_to_c_from_l)

    return R_to_c_from_l


def interp_transforms(T1, T2, alpha):
    assert alpha <= 1
    T_avg = alpha * T1 + (1 - alpha) * T2
    q1 = quaternion_from_matrix(T1)
    q2 = quaternion_from_matrix(T2)
    q3 = quaternion_slerp(q1, q2, alpha)
    R = quaternion_matrix(q3)
    T_avg[0:3, 0:3] = R[0:3, 0:3]
    return T_avg


def interp_transforms_backward(imu_transforms, ind):
    assert ind < 0, 'No need to call interp_transforms_backward'
    T_interp = imu_transforms[0, :, :] -\
        (imu_transforms[-1 * ind, :, :] - imu_transforms[0, :, :])
    R = T_interp[0:3, 0:3]
    R = np.linalg.qr(R, mode='complete')[0]
    T_interp[0:3, 0:3] = R
    return T_interp


def transform_points_in_sweep(pts, times, fnum, imu_transforms):
    for time in set(times):
        mask = times == time

        # FIXME PARAM
        offset = (time / float(1e6)) / 0.05
        offset = min(5, offset)

        ind1 = int(fnum - math.ceil(offset))
        ind2 = int(fnum - math.floor(offset))

        # FIXME Hack to interpolate before 0
        #ind1, ind2 = max(ind1, 0), max(ind2, 0)
        if ind1 < 0:
            T1 = interp_transforms_backward(imu_transforms, ind1)
        else:
            T1 = imu_transforms[ind1, :, :]
        if ind2 < 0:
            T2 = interp_transforms_backward(imu_transforms, ind2)
        else:
            T2 = imu_transforms[ind2, :, :]

        transform = interp_transforms(T1, T2, offset / 5.0)

        # transform data into imu_0 frame
        pts[:, mask] = np.dot(transform, pts[:, mask])


def transform_points_by_times(pts, t_pts, imu_transforms, gps_times):
    for t in set(t_pts):
        mask = t_pts == t

        fnum1 = bisect.bisect(gps_times, t) - 1
        fnum2 = fnum1 + 1
        try:
            alpha = (1 - (t - gps_times[fnum1])) / float(gps_times[fnum2] -
                                                         gps_times[fnum1])
        except IndexError:
            continue

        T1 = imu_transforms[fnum1, :, :]
        T2 = imu_transforms[fnum2, :, :]

        transform = interp_transforms(T1, T2, alpha)

        # transform data into imu_0 frame
        pts[:, mask] = np.dot(transform, pts[:, mask])


if __name__ == '__main__':
    import sys
    z = loadLDRCamMap(sys.argv[1])
    for x in z:
        print x
