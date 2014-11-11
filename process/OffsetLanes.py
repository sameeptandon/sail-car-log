#!/usr/bin/python
# -*- coding: utf-8 -*-
#Usage: OffsetLanes.py interpolated_lanes.pickle num_left num_right lidar.npz output_folder/

import sys
from MultiLaneGenerator import MultiLane
import numpy as np

def saveInterp(folder, interp, num_left, num_right, name='multilane_points'):
    num_lanes = num_left + num_right
    out = {}
    out['num_lanes'] = np.array(num_lanes)
    out['num_left'] = np.array(num_left)
    out['num_right'] = np.array(num_right)
    for i in xrange(num_lanes):
        out['lane' + str(i)] = interp[:,:,i]

    f = folder + '/' + name
    print 'Saved', f
    np.savez(f, **out)

if __name__ == '__main__':
    left = int(sys.argv[2])
    right = int(sys.argv[3])
    ml = MultiLane(sys.argv[1], left, right, sys.argv[4])
    ml.extendLanes()
    # ml.offsetInterpolated()
    if len(sys.argv) == 6:
        saveInterp(sys.argv[5], ml.interp, left, right)
    else:
        saveInterp(sys.argv[5], ml.interp, left, right, sys.argv[6])
