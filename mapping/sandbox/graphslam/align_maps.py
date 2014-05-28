import os
import argparse
import numpy as np
import h5py
from chunk_and_align import get_enu0
from graphslam_config import REALIGN_EVERY
from pipeline_config import EXPORT_STEP
from chunk_and_align import get_closest_key_value

'''
Take the results of chunk_and_align and apply them to points
exported prior to projecting them onto the videos
'''

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='apply alignments to lidar points')
    parser.add_argument('ldr1', help='npz file with lidar points, run 1')
    parser.add_argument('ldr2', help='npz file with lidar points, run 2')
    parser.add_argument('gps1', help='gps file 1')
    parser.add_argument('gps2', help='gps file 2')
    parser.add_argument('match_file', help='h5 file w/ matches')
    parser.add_argument('t1', help='imu transforms, run 1')
    parser.add_argument('t2', help='imu transforms, run 2')
    parser.add_argument('tb', help='file with transform between runs')
    parser.add_argument('align_data', help='alignment data for projecting map on video')
    parser.add_argument('map_data1', help='map data for doing alignment evaluation')
    parser.add_argument('map_data2', help='map data for doing alignment evaluation')
    args = parser.parse_args()

    # Read in lidar data and transforms

    print args.gps1, args.gps2
    enu1 = get_enu0(args.gps1, args.gps1)
    enu2 = get_enu0(args.gps2, args.gps1)

    print args.t1, args.t2
    imu_transforms1 = np.load(args.t1)['data']
    imu_transforms2 = np.load(args.t2)['data']

    print args.ldr1, args.ldr2
    all_data1 = np.load(args.ldr1)['data']
    all_data2 = np.load(args.ldr2)['data']

    h5f = h5py.File(args.match_file, 'r')
    nn_matches = h5f['matches'][...]
    h5f.close()
    start2, start1 = nn_matches[0, :]
    start1 = start1 / EXPORT_STEP
    start2 = start2 / EXPORT_STEP
    nn_dict = dict()
    for t in range(nn_matches.shape[0]):
        nn_dict[nn_matches[t, 1]] = nn_matches[t, 0]

    Ts = list()
    chunk_num = 0
    for k in range(start1, start1 + nn_matches.shape[0] / EXPORT_STEP, REALIGN_EVERY):
        f = os.path.splitext(args.tb)[0] + '--%d' % chunk_num + '.h5'
        print f
        if not os.path.exists(f):
            break
        try:
            h5f = h5py.File(f, 'r')
        except IOError, e:
            print 'Failed opening:', f
            raise
        T = h5f['transform'][...]
        Ts.append(T)
        chunk_num += 1
        h5f.close()

    # Apply transform in global coordinates

    all_data2[:, 0:3] = all_data2[:, 0:3] + (enu2 - enu1)

    # Transforms computed by scan matching

    chunk_num = 0
    #all_data2[:, 0:3] = all_data2[:, 0:3] + tbs[0][0:3, 3]
    for k in range(start1, start1 + nn_matches.shape[0] / EXPORT_STEP, REALIGN_EVERY):
        start_ind = k * EXPORT_STEP
        t_start = get_closest_key_value(k * EXPORT_STEP, nn_dict, max_shift=10)
        t_final = get_closest_key_value(k * EXPORT_STEP + REALIGN_EVERY * EXPORT_STEP, nn_dict, max_shift=10)
        mask_window = (all_data2[:, 4] < t_final) & (all_data2[:, 4] >= t_start)
        all_data2_mean = np.mean(all_data2[mask_window, 0:3], axis=0)
        all_data2[mask_window, 0:3] = np.dot(Ts[chunk_num][0:3, 0:3],
                (all_data2[mask_window, 0:3] - all_data2_mean).T).T + all_data2_mean + Ts[chunk_num][0:3, 3]
        #all_data2[mask_window, 0:3] = np.dot(np.eye(3),
                #(all_data2[mask_window, 0:3] - all_data2_mean).T).T + all_data2_mean + Ts[chunk_num][0:3, 3]
        '''
        for t in range(t_start, t_final):
            if chunk_num == len(Ts) - 1:
                # Can't interpolate
                tb = tbs[chunk_num][0:3, 3]
            else:
                # Interpolate
                tb = tbs[chunk_num][0:3, 3] * (t_final - t) / float(t_final - t_start) + tbs[chunk_num + 1][0:3, 3] * (t - t_start) / float(t_final - t_start)
            mask = all_data2[:, 4] == t
            all_data2[mask, 0:3] = all_data2[mask, 0:3] + tb
        '''
        chunk_num += 1
        if chunk_num >= len(Ts):
            break

    # Write data for projecting onto video

    np.savez(args.align_data, start1=start1, start2=start2, all_data1=all_data1, all_data2=all_data2,
            imu_transforms1=imu_transforms1, imu_transforms2=imu_transforms2, nn_matches=nn_matches)

    # Also write data for doing alignment evaluation

    h5f = h5py.File(args.map_data1, 'w')
    dset1 = h5f.create_dataset('points', all_data1.shape, dtype='f')
    dset1[...] = all_data1
    h5f.close()

    h5f = h5py.File(args.map_data2, 'w')
    dset2 = h5f.create_dataset('points', all_data2.shape, dtype='f')
    dset2[...] = all_data2
    h5f.close()
