import h5py
import numpy as np
from Q50_config import LoadParameters
from pipeline_config import PARAMS_FILE, PARAMS_H5_FILE

def get_param(params, key):
    parts = key.split('/')
    param = params[parts[0]]
    for part in parts[1:]:
        if part.isdigit():
            param = param[int(part)]
        else:
            param = param[part]
    return param

if __name__ == '__main__':
    params = LoadParameters(open(PARAMS_FILE, 'r').readline().rstrip())
    #import IPython; IPython.embed()
    keys = params.keys()

    h5f = h5py.File(PARAMS_H5_FILE, 'w')

    for key in keys:
        val = get_param(params, key)
        if isinstance(val, list) and isinstance(val[0], dict):
            for j in range(len(val)):
                keys.append(key + '/' + str(j))
        elif isinstance(val, dict):
            for subkey in val:
                keys.append(key + '/' + subkey)
        else:
            print key
            if isinstance(val, np.ndarray):
                # Write matrix
                if len(val.shape) == 1:
                    val = val.reshape((val.shape[0], 1))
                h5f[key] = val
            elif isinstance(val, list):
                assert isinstance(val[0], (int, float))
                # Reshape so column vector
                h5f[key] = np.array(val).reshape((len(val), 1))
            else:
                assert isinstance(val, (int, float))
                h5f[key] = np.array(val)

    h5f.close()
