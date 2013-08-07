import numpy as np
import pdb
import sys
from scipy.io import loadmat, savemat


if __name__ == '__main__':
    labels = loadmat(sys.argv[1])
    left = labels['left']
    right = labels['right']

    polynomial_fit = 3
    boundary = 50

    outputs = [{}, {}]
    points_arr = [left, right]

    for point_pos in range(len(points_arr)):
        points = points_arr[point_pos]

        interpolated = np.copy(points)
        interpolated[np.where(interpolated[:, 1] == -1)[0]] = np.array([0, 0])

        data_points = np.zeros(points.shape[0])
        num_labels = points.shape[0]

        start = 0
        while start < num_labels and points[start, 1] == -1:
            start += 1

        """
        # extend first recorded point backwards in time
        first_point = points[start]
        interpolated[0:start] = first_point
        interpolated[0:start, 4] = range(0, start)
        """

        next_point = start + 1

        while next_point < num_labels:
            # continue until you find a known point followed by an unknown point
            while next_point < num_labels and points[next_point, 1] != -1:
                start += 1
                next_point += 1

            while next_point < num_labels and points[next_point, 1] == -1:
                next_point += 1

            if next_point == num_labels:
                break

            lowest_point = max(start - boundary, 0)
            highest_point = min(next_point + boundary + 1, num_labels)

            time_array = lowest_point + np.where(points[lowest_point:highest_point, 1] != -1)[0]

            pos = points[time_array, 0:3]
            p = np.polyfit(time_array, pos, polynomial_fit)
            """x = points[time_array, 0]
            y = points[time_array, 1]
            z = points[time_array, 2]
            p_x = np.polyfit(time_array, x, polynomial_fit)
            p_y = np.polyfit(time_array, y, polynomial_fit)
            p_z = np.polyfit(time_array, z, polynomial_fit)"""

            for t in xrange(start, next_point):
                if t not in time_array:
                    interp = np.copy(p[polynomial_fit])
                    for x in xrange(polynomial_fit):
                        interp += p[x] * t**(polynomial_fit - x)
                    interpolated[t] = np.array([interp[0], interp[1]])
                    data_points[t] += 1

            start = next_point

        """
        if start != num_labels:
            last_point = points[start]
            interpolated[start+1:] = last_point
            interpolated[start+1:, 4] = range(start+1, num_labels)
        """

        data_points[data_points == 0] = 1
        #interpolated = (interpolated.transpose() / data_points.astype(np.float32)).transpose()

        outputs[point_pos] = interpolated

    savemat(sys.argv[2], {'left': outputs[0], 'right': outputs[1]})
