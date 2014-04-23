import numpy as np
import sys
import h5py
from transformations import euler_from_matrix

if __name__ == '__main__':
    in_file = sys.argv[1]
    out_file = sys.argv[2]

    h5f = h5py.File(in_file, 'r')
    transform = h5f['transform'][...]
    euler = np.array(euler_from_matrix(transform))
    h5f.close()

    h5f = h5py.File(out_file, 'w')
    euler_shape = (euler.shape[0], 1)
    dset = h5f.create_dataset('euler', euler_shape, dtype='f')
    dset[...] = euler.reshape(euler_shape)
    h5f.close()
