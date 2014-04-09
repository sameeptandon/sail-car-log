import numpy as np
import os

def loadRDR(rdrfile):
    # OBJ_fmt = 'O {id} {dist} {lat_dist} {rel_spd} {dyn_prop} {rcs} {w} {l}'
    # TGT_fmt = 'T {id} {dist} {lat_dist} {rel_spd} {dyn_prop} {traj} {w} {l} {obst_probab} {exist_probab} {rel_acc} {type} {lost_reason}'
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