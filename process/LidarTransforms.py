import os
import glob
import numpy as np

# PARAM
SWEEP_TIME_MICROSEC = 100000


def loadLDR(ldrfile):
    z = np.fromfile(ldrfile, dtype=np.float32)
    z = z.reshape((z.shape[0] / 6, 6))
    return z


def loadPCD(pcdfile): 
    f = open(pcdfile, 'r')
    count = 0
    data_started = False
    for l in f:
        t = l.split()
        if t[0] == 'POINTS':
            num_pts = int(t[1])
            pts = np.zeros((num_pts, 4), float)
            continue
        elif t[0] == 'DATA':
            data_started = True
            continue
        elif data_started:
            z = np.array(map(lambda x: float(x), t))
            pts[count,:] = z
            count += 1

    return pts


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


class LDRLoader(object):

    def __init__(self, ldr_dir):
        ldr_files = sorted(list(glob.glob(os.path.join(ldr_dir, '*.ldr'))))
        file_times = np.array([int(os.path.splitext(os.path.basename(f))[0]) for f in ldr_files], dtype=np.int64)
        sweep_end_times = file_times
        sweep_start_times = sweep_end_times[:-1]
        sweep_start_times = np.insert(sweep_start_times, 0,
                sweep_start_times[0] - SWEEP_TIME_MICROSEC)
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
            times = -1 * data[:, 5] + sweep_end_time

            if all_data is None:
                all_data = data
                all_times = times
            else:
                all_data = np.vstack((all_data, data))
                all_times = np.concatenate((all_times, times))

        # Filter points within time window

        mask = (all_times >= microsec_since_epoch - time_window / 2.0) &\
               (all_times <= microsec_since_epoch + time_window / 2.0)
        return all_data[mask, :]


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

if __name__ == '__main__':
    import sys
    z = loadLDRCamMap(sys.argv[1])
    for x in z:
        print x
