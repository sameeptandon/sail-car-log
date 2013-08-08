import numpy as np
import pdb
import sys
from scipy.io import loadmat, savemat
import matplotlib.pyplot as mplot
import scipy.interpolate

if __name__ == '__main__':
    labels = loadmat(sys.argv[1])
    left = labels['left']
    right = labels['right']

    polynomial_fit = 3
    boundary = 150

    outputs = [{}, {}]
    points_arr = [left, right]

    left_time_array = np.where(left[:, 0] != -1)[0]
    right_time_array = np.where(right[:, 0] != -1)[0]

    spline_left_x = scipy.interpolate.UnivariateSpline(left_time_array, left[left_time_array, 0])
    spline_left_y = scipy.interpolate.UnivariateSpline(left_time_array, left[left_time_array, 1])
    spline_right_x = scipy.interpolate.UnivariateSpline(right_time_array, right[right_time_array, 0])
    spline_right_y = scipy.interpolate.UnivariateSpline(right_time_array, right[right_time_array, 1])
    
    all_time = np.arange(0, left.shape[0])

    output_left = np.copy(left);
    output_left[:,0] = spline_left_x(all_time);
    output_left[:,1] = spline_left_y(all_time); 

    output_right = np.copy(right);
    output_right[:,0] = spline_right_x(all_time);
    output_right[:,1] = spline_right_y(all_time); 

    savemat(sys.argv[2], {'left': output_left, 'right':output_right})
    #mplot.scatter(all_time, left[:,0])
    #mplot.scatter(all_time, spline_left_x(all_time), alpha=0.1)
    #mplot.show()
