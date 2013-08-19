import sys, os, cv2, cv, random
import numpy as np
import pickle

def loadFiles(target_dir, filter_fn): 
    files = os.listdir(target_dir)
    target_files = filter(filter_fn, files)
    target_files.sort()
    ret_vals = [ ]
    for p in range(len(target_files)):
        val = pickle.load(open(target_dir + '/' + target_files[p]))
        ret_vals.append(val)
    return ret_vals

if __name__ == '__main__':
    # load data
    left_pts = loadFiles(sys.argv[1], lambda x: 'lpt' in x)
    right_pts = loadFiles(sys.argv[1], lambda x: 'rpt' in x)

    # flatten list
    left_data = np.array([item for sublist in left_pts for item in sublist], np.float32)
    right_data = np.array([item for sublist in right_pts for item in sublist], np.float32)

    # subsample 60 images
    indx = sorted(random.sample(xrange(len(left_data)), 40)) 
    left_data = [ left_data[i] for i in indx ] 
    right_data = [ right_data[i] for i in indx ] 
    
    # generate obj_pts
    patternShape = (8,12)
    pattern_points = np.zeros((np.prod(patternShape), 3), np.float32)
    pattern_points[:,:2] = np.indices(patternShape).T.reshape(-1,2)
    left_obj_pts = [ ] 
    for j in range(len(left_data)):
        left_obj_pts.append(np.copy(pattern_points))
    right_obj_pts = [ ] 
    for j in range(len(right_data)):
        right_obj_pts.append(np.copy(pattern_points))


    left_camera_matrix = np.array([[2200.0, 0.0, 620.0],
                                   [0.0, 2200.0, 440.0],
                                   [0.0, 0.0, 1.0]])
    left_dist_coeffs = np.zeros(4)
    # calibrate left camera
    print 'Calibrating Left Camera...'
    left_rms = cv2.calibrateCamera(left_obj_pts, left_data, (1280, 960), left_camera_matrix, left_dist_coeffs, flags=cv.CV_CALIB_USE_INTRINSIC_GUESS)
    right_camera_matrix = np.array([[2200.0, 0.0, 620.0],
                                   [0.0, 2200.0, 440.0],
                                   [0.0, 0.0, 1.0]])
    right_dist_coeffs = np.zeros(4)
    # calibrate right camera
    print 'Calibrating Right Camera...'
    right_rms = cv2.calibrateCamera(right_obj_pts, right_data, (1280, 960), right_camera_matrix, right_dist_coeffs, flags=cv.CV_CALIB_USE_INTRINSIC_GUESS)
    print left_camera_matrix
    # calibrate stereo
    R = np.eye(3)
    T = np.zeros(3)
    print 'Calibrating Stereo...'
    retvals = cv2.stereoCalibrate(left_obj_pts, left_data, right_data, left_camera_matrix, left_dist_coeffs, right_camera_matrix, right_dist_coeffs, (1280,960), R, T )
