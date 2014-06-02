import sys
import numpy as np
from LidarTransforms import loadLDR
from sklearn.neighbors import NearestNeighbors

MAX_INTERP_DIST = 0.5


def interp_ldr(ldr0, ldr1, inds0, inds1, ni=1):
    num_points = inds0.size * ni
    ldr_interp = np.zeros((num_points, 6), dtype=np.float32)
    for k in range(ni):
        # Average x, y, z, intensity
        ldr_interp[k::ni, 0:4] = ((ni - k) * ldr0[inds0, 0:4] +
                (k + 1.0) * ldr1[inds1, 0:4]) / (ni + 1.0)
        # Average the time as well
        ldr_interp[k::ni, 5] = ((ni + - k) * ldr0[inds0, 5] +
                (k + 1.0) * ldr1[inds1, 5]) / (ni + 1.0)
        # Concatenate the laser numbers
        ldr_interp[k::ni, 4] = ldr0[inds0, 4] * 100 + ldr1[inds1, 4]
    return ldr_interp


if __name__ == '__main__':
    infile = sys.argv[1]
    outfile = sys.argv[2]
    ni = int(sys.argv[3])
    
    ldr_data = loadLDR(infile)

    laser_nums = ldr_data[:, 4]
    # Laser num goes from 0 to 31
    print np.min(laser_nums), np.max(laser_nums)

    laser_data = list()
    laser_nbrs = list()
    for laser_num in range(32):
        ld = ldr_data[laser_nums == laser_num, :]
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(ld[:, 0:3])
        laser_data.append(ld)
        laser_nbrs.append(nbrs)

    interp_data = list()
    # Refer to HDL-32E manual for laser ordering
    for laser_num in range(0, 30):
        ld0 = laser_data[laser_num]
        ld1 = laser_data[laser_num + 2]
        inds1 = np.array(range(ld1.shape[0]))
        dists, inds0 = laser_nbrs[laser_num].kneighbors(ld1[:, 0:3])
        mask = dists.ravel() < MAX_INTERP_DIST
        inds0 = inds0[mask].ravel()
        inds1 = inds1[mask]
        ld_interp = interp_ldr(ld0, ld1, inds0, inds1, ni=ni)
        interp_data.append(ld_interp)

    interp_data = np.vstack(interp_data)

    print 'original:', ldr_data.shape
    print 'interped:', interp_data.shape

    combined_data = np.vstack((ldr_data, interp_data))
    print 'combined:', combined_data.shape
    combined_data.tofile(outfile)
