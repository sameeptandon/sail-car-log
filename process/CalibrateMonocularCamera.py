import sys, os, cv2, cv, random
import numpy as np
from transformations import euler_from_matrix

patternShape = (10,7)
#patternShape = (12, 8)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

def loadFiles(target_dir, filter_fn): 
    files = os.listdir(target_dir)
    target_files = filter(filter_fn, files)
    target_files.sort()
    ret_vals = [ ]
    for p in range(len(target_files)):
        val = cv2.imread(target_dir + '/' + target_files[p])
        ret_vals.append(val)
    return ret_vals

if __name__ == '__main__':
    # load images
    left_imgs = loadFiles(sys.argv[1], lambda x: 'png' in x)
    for idx in range(len(sys.argv)):
        if idx >= 2:
            left_imgs.append(cv2.imread(sys.argv[idx]))
    #fixed_board = cv2.imread(sys.argv[2])
    #left_imgs.append(fixed_board)
    print len(left_imgs)
    

    left_corners = None
    left_img_points = [ ]
    left_cb_imgs = [ ]
    # extract checkerboards
    for img in left_imgs: 
        flags = cv2.CALIB_CB_FAST_CHECK
        I_left = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret_left, left_corners = cv2.findChessboardCorners(I_left, patternShape, None)#, flags=flags)

        if ret_left == False:
            print 'no checkerboard found??'
            continue

        cv2.cornerSubPix(I_left, left_corners, (10,10), (-1, -1), criteria)
        I_left_cb = img.copy()

        cv2.drawChessboardCorners(I_left_cb, patternShape, left_corners, ret_left)
        cv2.imshow('left', cv2.pyrDown(I_left_cb))
        key = chr((cv2.waitKey(5) & 255))
        left_img_points.append(left_corners.reshape(-1,2))
        left_cb_imgs.append(img.copy())



    print len(left_cb_imgs)
    # flatten list
    #left_data = np.array([item for sublist in left_pts for item in sublist], np.float32)

    # subsample 60 images
    left_data = left_img_points 

    # generate obj_pts
    pattern_points = np.zeros((np.prod(patternShape), 3), np.float32)
    pattern_points[:,:2] = np.indices(patternShape).T.reshape(-1,2) * 0.099
    left_obj_pts = [ ] 
    for j in range(len(left_data)):
        left_obj_pts.append(np.copy(pattern_points))
    
    # calibrate left camera
    print 'Calibrating Left Camera...'
    print left_imgs[0].shape
    rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(left_obj_pts, left_data, (left_imgs[0].shape[0], left_imgs[0].shape[1]))
    print rms
    print cameraMatrix
    print distCoeffs
    print tvecs

    for idx in range(len(left_img_points)):
        num = idx
        r = rvecs[idx]
        print r
        imgpoints2, _ = cv2.projectPoints(np.copy(pattern_points), r, tvecs[idx], cameraMatrix, distCoeffs)
        draw_I = left_cb_imgs[idx].copy()
        pix = imgpoints2.transpose()
        pix = np.around(pix[:,0,:])
        pix = pix.astype(np.int32)
        for p in range(12):
            draw_I[pix[1,:]+p, pix[0,:]] = [0, 5, 255]
            draw_I[pix[1,:]-p, pix[0,:]] = [0, 5, 255]
            draw_I[pix[1,:], pix[0,:]+p] = [0, 5, 255]
            draw_I[pix[1,:], pix[0,:]-p] = [0, 5, 255]
        cv2.imshow('reproj', cv2.pyrDown(draw_I))
        cv2.waitKey(50)

    """
    print tvecs[-1]
    print rvecs[-1]
    print cv2.Rodrigues(rvecs[-1])
    print euler_from_matrix(cv2.Rodrigues(rvecs[-1])[0], 'sxyz')
    """
    





