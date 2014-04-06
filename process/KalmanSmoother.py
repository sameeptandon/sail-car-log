import numpy as np
from numpy.linalg import inv

'''
Kalman smoothing of trajectories using INS transforms combined with
ICP scan matching
'''


def kf_pass(A0, B0, C1, d1, Q0, R1, mu00, Sigma00, u0, z1):
    # Kalman filter forward pass
    # Follows convention:
    # x1 = A0*x0 + B0*u0 + eta,   eta ~ N(0, Q0)
    # z1 = C1*x1 + d1 + delta,  delta ~ N(0, R1)

    # Dynamics update
    mu10 = np.dot(A0, mu00) + np.dot(B0, u0)
    Sigma10 = np.dot(np.dot(A0, Sigma00), A0.T) + Q0

    # Measurement update
    if z1 is not None:
        K = np.dot(np.dot(Sigma10, C1.T), inv(np.dot(np.dot(C1, Sigma10), C1.T) + R1))
        mu11 = mu10 + np.dot(K, (z1 - np.dot(C1, mu10) + d1))
        Sigma11 = np.dot(np.eye(A0.shape[0]) - np.dot(K, C1), Sigma10)
    else:
        mu11 = mu10
        Sigma11 = Sigma10

    # Return mu10 and Sigma10 as well for smoothing
    return (mu10, Sigma10, mu11, Sigma11)


def ks_pass(A, mu00, mu10, mu1T, Sigma00, Sigma10, Sigma1T):
    # Kalman smoother backward pass
    L = np.dot(np.dot(Sigma00, A.T), inv(Sigma10))
    mu0T = mu00 + np.dot(L, (mu1T - mu10))
    Sigma0T = Sigma00 + np.dot(np.dot(L, Sigma1T - Sigma10), L.T)
    return (mu0T, Sigma0T)


def plot_ks_results(mus, Tmus, Sigmas, TSigmas, imu_states, coord=0):
    # Plots to compare GPS positions to positions after filtering and smoothing
    import matplotlib.pyplot as plt

    cn = ['x', 'y', 'z'][coord]

    ys1 = np.array([mu[coord] for mu in mus])
    ys2 = np.array([mu[coord] for mu in Tmus])
    ys3 = np.array(imu_states[:, coord])

    sigmas = np.array([Sigma[coord, coord] for Sigma in Sigmas])
    Tsigmas = np.array([Sigma[coord, coord] for Sigma in TSigmas])

    plt.figure()

    # Original GPS positions
    plt.plot(range(nt), ys3, 'r-', label='$%s\mathrm{-GPS}$' % cn)

    # Kalman filter results
    plt.plot(range(nt), ys1, 'b-', label='$%s\mathrm{-filtered}$' % cn)
    # Plot variance interval as well -- should be periodic
    plt.plot(range(nt), ys1 + sigmas, 'b:',
            label='$%s\mathrm{-filtered\ } \pm\ \Sigma_{%s%s}$' % (cn, cn, cn))
    plt.plot(range(nt), ys1 - sigmas, 'b:')

    # Kalman smoother results
    plt.plot(range(nt), ys2, 'g-', label='$%s\mathrm{-smoothed}$' % cn)
    plt.plot(range(nt), ys2 + Tsigmas, 'g:',
            label='$%s\mathrm{-smoothed\ } \pm\ \Sigma_{%s%s}$' % (cn, cn, cn))
    plt.plot(range(nt), ys2 - Tsigmas, 'g:')

    handles, labels = plt.gca().get_legend_handles_labels()
    plt.legend(handles, labels, prop={'size': 20}, loc=0)
    plt.show()


if __name__ == '__main__':
    import os
    import sys
    import h5py
    from ArgParser import parse_args
    from GPSReader import GPSReader
    from GPSTransforms import IMUTransforms
    from pipeline_config import ICP_TRANSFORMS_DIR, EXPORT_START, EXPORT_NUM,\
        EXPORT_STEP

    args = parse_args(sys.argv[1], sys.argv[2])

    # Load INS data

    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)

    T_start = EXPORT_START
    T_end = T_start + EXPORT_NUM * EXPORT_STEP
    nt = T_end - T_start
    imu_states = imu_transforms[T_start:T_end, 0:3, 3]

    # Load ICP transform corrections

    icp_transforms = list()
    for t in range(1, nt / EXPORT_STEP):
        h5_file = os.path.join(ICP_TRANSFORMS_DIR, '%d.h5' % t)
        print h5_file
        h5f = h5py.File(h5_file, 'r')
        transform = h5f['transform'][...]
        h5f.close()
        icp_transforms.append(transform)

    # Set up variables for Kalman smoothing

    # Same across all time steps - right now dynamics model is pretty dumb
    A = np.eye(3)
    B = np.eye(3)
    C = np.eye(3)
    R = np.diag([0.1, 0.1, 0.1])

    # Dependent on t
    us = list()
    ds = list()
    Qs = list()
    Qs.append(np.diag([0.01, 0.3, 0.3]))
    for t in range(1, nt):
        us.append(imu_states[t, :] - imu_states[t - 1, :])
        if t % EXPORT_STEP == 0:
            ds.append(icp_transforms[t / EXPORT_STEP - 1][0:3, 3])
        else:
            ds.append(None)
        Qs.append(np.diag(np.abs(imu_states[t, :] - imu_states[t - 1, :])))

    # Run Kalman filter

    mus = list()      # mu_{t|t}
    Sigmas = list()   # Sigma_{t|t}
    dmus = list()     # mu_{t+1|t}
    dSigmas = list()  # Sigma_{t+1|t}
    mu00 = imu_states[0, :]
    Sigma00 = Qs[0]
    mus.append(mu00)
    Sigmas.append(Sigma00)

    for t in range(1, nt):
        z = None
        d = None
        u = us[t - 1]

        if t % EXPORT_STEP == 0:
            # Get an ICP observation
            z = imu_states[t, :]
            d = ds[t - 1]
            assert d is not None

        dm, dSig, m, Sig = kf_pass(A, B, C, d, Qs[t], R, mus[-1], Sigmas[-1], u, z)

        mus.append(m)
        Sigmas.append(Sig)
        dmus.append(dm)
        dSigmas.append(dSig)

    # Run Kalman smoothing

    Tmus = list()     # mu_{t|T}
    TSigmas = list()  # Sigma_{t|T}
    Tmus.append(mus[-1])
    TSigmas.append(Sigmas[-1])

    for t in range(nt - 2, -1, -1):
        Tm, TSig = ks_pass(A, mus[t], dmus[t], Tmus[0], Sigmas[t], dSigmas[t], TSigmas[0])

        # Prepend
        Tmus.insert(0, Tm)
        TSigmas.insert(0, TSig)

    # Plot stuff

    plot_ks_results(mus, Tmus, Sigmas, TSigmas, imu_states, coord=2)

    # Export to file
