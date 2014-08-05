import numpy as np
import os
import glob

# PARAM
SWEEP_TIME_MICROSEC = 60000

def loadRDR(rdrfile):
    T_pts = []
    O_pts = []
    for line in open(rdrfile):
        line = line.rstrip()
        # Assuming that tgt messages happen after obj messages
        tag = line[0]
        data = line[2:].rstrip()
        tokens = [float(e) for e in data.split(' ')]
        if tag == 'T':
            (id, dist, lat_dist, rel_spd, dyn_prop, traj, w, l,
                obst_probab, exist_probab, rel_acc, type, lost_reason) = tokens
            T_pts.append([dist, lat_dist, 0, l, w])
        else:
            (id, dist, lat_dist, rel_spd, dyn_prop, rcs, w, l) = tokens
            O_pts.append([dist, lat_dist, 0, l, w, rcs, rel_spd, id])

    return (np.array(O_pts), np.array(T_pts))

def loadRDRCamMap(frame_cloud_map):
    map_file = open(frame_cloud_map, 'r')
    points = []
    frame_folder, map_name = os.path.split(frame_cloud_map)
    frame_folder = frame_folder + '/' + map_name.split('.')[0] + '_rdr'

    for line in map_file:
        tokens = line.rstrip().split(' ')
        if len(tokens) == 3:
            rdr_file = line.rstrip().split(' ')[2]
            points.append(frame_folder + '/' + rdr_file)
        else:
            map_file.close()
            return None

    map_file.close()
    return points

def calibrateRadarPts(pts, params):
    #T_pts[id] = [dist + 3.17, lat_dist + .4, -1.64, l, w]
    R = params['R_from_r_to_l']

    pts[:, 0] += params['T_from_r_to_l'][0]
    pts[:, 1] += params['T_from_r_to_l'][1]
    pts[:, 2] += params['T_from_r_to_l'][2]

    return np.dot(R, pts.transpose()).transpose()

class RDRLoader(object):
    def __init__(self, rdr_dir):
        rdr_files = sorted(list(glob.glob(os.path.join(rdr_dir, '*.rdr'))))
        file_times = [int(os.path.splitext(os.path.basename(f))[0]) for f in
                      rdr_files]
        file_times = np.array(file_times, dtype=np.int64)
        self.rdr_end_times = file_times
        self.rdr_start_times = self.rdr_end_times - SWEEP_TIME_MICROSEC
        self.rdr_files = np.array(rdr_files)

    def loadRDRWindow(self, microsec_since_epoch, fmt='O'):
        """
        Loads a window of radar outputs closest to microseconds_since_epoch
        fmt can be 'O', 'T', or 'OT'.
        'O' returns raw objects - This should be used most of the time
        'O' format = id, dist, lat_dist, rel_spd, dyn_prop, rcs, w, l

        'T' returns target objects - These rarely occur, but give more
            information than 'O' objects
        'T' format = id, dist, lat_dist, rel_spd, dyn_prop, traj, w, l,
                     obst_probab, exist_probab, rel_acc, type, lost_reason

        'OT' - returns a tuple of both 'O' and 'T' objects
        """
        start_time = microsec_since_epoch - SWEEP_TIME_MICROSEC
        end_time = microsec_since_epoch + SWEEP_TIME_MICROSEC
        mask = (self.rdr_start_times >= start_time) & \
               (self.rdr_end_times <= end_time)

        O_pts = None
        T_pts = None

        rdr_files = self.rdr_files[mask].tolist()

        if len(rdr_files) > 0:
            rdr_file = rdr_files[len(rdr_files)/2]
            (O, T) = loadRDR(rdr_file)
            if fmt == 'O' or fmt == 'OT':
                O_pts = O
            elif fmt == 'T' or fmt == 'OT':
                T_pts = T

        if fmt == 'O':
            return O_pts
        elif fmt == 'T':
            return T_pts
        elif fmt == 'OT':
            return (O_pts, T_pts)

if __name__ == "__main__":
    import sys
    loader = RDRLoader(sys.argv[1])
    print loader.loadRDRWindow(1397083351092127, 1)
