import os
import numpy as np
import gurobipy
from gurobipy import GRB
from numpy.linalg import inv
from WGS84toENU import WGS84toENU
from WGS84toENU import deg2rad
from GPSTransforms import integrateVelocity
from graphslam_config import BIAS_GAMMA, dt
from gps_viewer import read_gps_fields
from GPSTransforms import R_to_i_from_w


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Solve graphslam QP for positions')
    parser.add_argument('gps_file', help='gps file to read raw positions and velocities from')
    parser.add_argument('ref_gps_file', help='gps file to read for llh0 in conversion to ENU')
    parser.add_argument('out_file', help='file to save opt positions to')
    parser.add_argument('--debug', action='store_true', help='display plots or not')
    args = parser.parse_args()

    # NOTE We're reading un ENU order while by default it's in NEU
    gps_fields = read_gps_fields(args.gps_file,
            ['lat', 'long', 'height', 'v_east', 'v_north', 'v_up'])
    ref_gps_fields = read_gps_fields(args.ref_gps_file, ['lat', 'long', 'height'])
    llh = np.array(gps_fields[0:3], dtype=np.float64).T
    llh_ref = np.array(ref_gps_fields, dtype=np.float64).T
    vel = np.array(gps_fields[3:6], dtype=np.float64)
    itg_vel = integrateVelocity(vel.T)
    xyz = WGS84toENU(llh_ref[0, :], llh)
    N = xyz.shape[1]
    #N = 10000

    model = gurobipy.Model('graphslam_qp')

    # Create variables

    # Positions
    xs = np.zeros((3, N), dtype=object)
    # Biases
    bs = np.zeros((3, N), dtype=object)
    for t in xrange(N):
        for k in xrange(3):
            xs[k, t] = model.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS)
            # PARAM, GPS is never that far off
            bs[k, t] = model.addVar(lb=-2.0, ub=2.0, vtype=GRB.CONTINUOUS)
    model.update()

    # No constraints
    # TODO Could think of other ways to formulate the optimization problem using slacks...

    # Objective

    R = np.diag([0.1, 0.1, 0.1])  # odometry
    Tau = np.diag([0.5, 0.5, 1.5])  # deviation from GPS w/ bias fix
    S = np.diag([0.5, 0.5, 0.5])  # bias

    terms = list()
    print 'Setting up the QP'
    print '- Adding odometry and bias terms'
    for t in xrange(1, N):
        d_odom = xs[:, t] - (xs[:, t-1] + vel[:, t-1] * dt)
        d_bias = bs[:, t] - BIAS_GAMMA * bs[:, t-1]
        terms.append(np.dot(np.dot(d_odom, inv(R)), d_odom))
        terms.append(np.dot(np.dot(d_bias, inv(S)), d_bias))
    print '- Adding GPS terms'
    for t in xrange(N):
        # NOTE bs[:, 0] not being penalized here except relative to bs[:, 1]
        d_gps = xs[:, t] - (xyz[:, t] + bs[:, t])
        terms.append(np.dot(np.dot(d_gps, inv(Tau)), d_gps))

    model.setObjective(gurobipy.quicksum(terms))

    # Optimize

    print 'Optimizing'
    model.optimize()
    opt_xs, opt_bs = np.zeros((3, N), dtype=np.float64), np.zeros((3, N), dtype=np.float64)
    for t in xrange(N):
        for k in range(3):
            opt_xs[k, t] = xs[k, t].x
            opt_bs[k, t] = bs[k, t].x

    # Plot

    import matplotlib.pyplot as plt
    coord = 2
    plt.plot(xyz[coord, :], label='gps')
    plt.plot(opt_xs[coord, :], label='graphslam')
    smooth_xyz = xyz[:, 0].reshape(3, 1) + itg_vel.T
    plt.plot(smooth_xyz[coord, :], label='smooth')
    #plt.plot(opt_bs[coord, :])

    handles, labels = plt.gca().get_legend_handles_labels()
    plt.legend(handles, labels, prop={'size': 20})

    plt.xlabel('t')
    plt.ylabel(['x', 'y', 'z'][coord])

    if args.debug:
        plt.show()
    else:
        # Save the result in 4x4 transform matrix format
        imu_transforms = np.zeros((N, 4, 4))
        rpy = read_gps_fields(args.gps_file, ['rot_x', 'rot_y', 'azimuth'])
        for t in range(N):
            imu_transforms[t, :, :] = np.eye(4)
            imu_transforms[t, 0:3, 3] = opt_xs[:, t]
            roll = deg2rad(rpy[0][t])
            pitch = deg2rad(rpy[1][t])
            yaw = -deg2rad(rpy[2][t])
            R = R_to_i_from_w(roll, pitch, yaw)
            imu_transforms[t, 0:3, 0:3] = R.transpose()

        # Save figure and transform data
        plt.savefig(os.path.splitext(args.out_file)[0] + '.pdf')
        np.savez(args.out_file, data=imu_transforms)
