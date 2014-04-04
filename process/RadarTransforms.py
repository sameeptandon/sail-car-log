import numpy as np
import os
from transformations import euler_matrix


def loadRDR(rdrfile):
    # OBJ_fmt = 'O {id} {dist} {lat_dist} {rel_spd} {dyn_prop} {rcs} {w} {l}'
    # TGT_fmt = 'T {id} {dist} {lat_dist} {rel_spd} {dyn_prop} {traj} {w} {l} {obst_probab} {exist_probab} {rel_acc} {type} {lost_reason}'
    T_pts = {}
    O_pts = {}
    for line in open(rdrfile):
        line = line.rstrip()
        # Assuming that tgt messages happen after obj messages
        tag = line[0]
        data = line[2:].rstrip()
        tokens = [float(e) for e in data.split(' ')]
        if tag == 'T':
            (id, dist, lat_dist, rel_spd, dyn_prop, traj, w, l,
                obst_probab, exist_probab, rel_acc, type, lost_reason) = tokens
            T_pts[id] = [dist, lat_dist, 0, l, w]
        else:
            (id, dist, lat_dist, rel_spd, dyn_prop, rcs, w, l) = tokens
            O_pts[id] = [dist, lat_dist, 0, l, w, rcs]
        
    return (np.array(O_pts.values()), np.array(T_pts.values()))

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

def calibrateRadarPts(pts, Rxyz=(0, 0, -.015), Txyz=(3.17, 0.4, -1.64)):
    #T_pts[id] = [dist + 3.17, lat_dist + .4, -1.64, l, w]
    (Tx, Ty, Tz) = Txyz
    (Rx, Ry, Rz) = Rxyz
    R = euler_matrix(Rx, Ry, Rz)[0:3,0:3]

    pts[:, 0] += Tx
    pts[:, 1] += Ty
    pts[:, 2] += Tz
    return np.dot(R, pts.transpose()).transpose()