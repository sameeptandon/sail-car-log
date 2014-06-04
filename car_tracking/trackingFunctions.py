import numpy as np
import cv2
import cv;

from matplotlib import pyplot as plt
import math
import sys
import copy
import os.path

from AnnotationLib import *

import pdb;

def filter_occluded(_rects):

    OCCLUSION_THRESHOLD = 0.4;

    rects = copy.deepcopy(_rects);

    # cars that are closer to the camera should come first -> closer to the camera means lower in the image
    #rects.sort(key=lambda r: r.width())
    rects.sort(key=lambda r: (r.y2, r.width()))

    rects_filtered = [];
    num_rects = len(rects);

    for ridx1, r in enumerate(rects):
        myarea = float(r.width() * r.height());
        do_filter = False;

        for ridx2 in range(ridx1+1, num_rects):
            intw, inth = r.intersection(rects[ridx2]);
            intarea = intw*inth;
            intratio = intw*inth / myarea;

            if intratio > OCCLUSION_THRESHOLD:
                do_filter = True;
                break;

        if not do_filter:
            rects_filtered.append(r);
        
    return rects_filtered;
    


def dist_chi2(h1, h2):
    res = 0; 

    for idx1 in range(h1.shape[0]): 
        for idx2 in range(h1.shape[1]): 
            d = h1[idx1, idx2] + h2[idx1, idx2]; 

            if d > 0:
                res += ((h1[idx1, idx2] - h2[idx1, idx2])**2) / d;

    return res;

def comp_rect_hist(I, _rect, normtype=1):

    rect = copy.deepcopy(_rect);
    rect.resize(0.9);

    roi_img = I[rect.y1:rect.y2, rect.x1:rect.x2, :];

    print "roi width: ", roi_img.shape[0], ", roi height: ", roi_img.shape[1];
    print rect.y1, rect.y2, rect.x1, rect.x2

    assert(roi_img.shape[0] > 0 and roi_img.shape[1] > 0);

    hsv_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV);
    dark = hsv_img[...,2] < 20

    hsv_img[dark] = 0

    hbins = 15;
    sbins = 15;

    roi_hist = cv2.calcHist( [hsv_img], [0, 1], None, [hbins, sbins], [0, 180, 0, 256])
    roi_hist[0, 0] = 0;

    # normalize 
    if normtype == 1:
        s = np.sum(roi_hist);
        if s > 0: 
            roi_hist /= s;
    else:
        assert(False);
        assert(normtype == 2);
        cv2.normalize(roi_hist, roi_hist, 1, 0, cv2.NORM_L2)

    return roi_hist;

def inc_image_name(s, n):
    pos1 = s.rfind('_');
    pos2 = s.rfind('.');

    pos1_slash = s.rfind('/');

    # supported formats: "/path/filename_<framenumber>.ext" or "/path/<framenumber>.ext" 
    if pos1 < pos1_slash:
        pos1 = pos1_slash;

    num = int(s[pos1+1:pos2]) + n
    numlen = pos2 - pos1 - 1;
    res = s[:pos1+1] + str(num).zfill(numlen) + s[pos2:];
    return res;

def convert_keypoints_opencv_vlfeat(list_kp):
	n = len(list_kp)
	
	kp = np.ndarray(shape=(n, 4), dtype=np.float32);

	for idx, k in enumerate(list_kp):
		kp[idx, 0] = k.pt[0];
		kp[idx, 1] = k.pt[1];
		kp[idx, 2] = k.size;
		kp[idx, 3] = k.angle;

	return kp;

def match_and_ratio_test(des1, des2): 
	# # BFMatcher with default params
        # bf = cv2.BFMatcher()
        # matches = bf.knnMatch(des1,des2, k=2)

	# if len(matches)<2:
	# 	return [];

        # # Apply ratio test
	# good = [m for m, n in matches if m.distance < 0.75*n.distance];

	# BFMatcher with default params
    
       	# MA: need at least two points to perform ratio test 
	if len(des1) <= 1 or len(des2) <= 1:
            return [];
        
	bf = cv2.BFMatcher()

	matches = bf.knnMatch(des1, des2, k=2)

	if len(matches)<2:
	 	return [];

	# Apply ratio test
	good = []
	for m,n in matches:
	    if m.distance < 0.75*n.distance:
		good.append(m)

	return good;

def SIFT(qImg, tImg, rect):

	# MA: test that keypoints are inside the region of interest rect
	# x1 = rect[0];
	# y1 = rect[1];
	# x2 = x1 + rect[2];
	# y2 = y1 + rect[3];

	x1 = rect.x1;
	y1 = rect.y1;
	x2 = rect.x2;
	y2 = rect.y2;

	use_opencv_sift = True;

	if use_opencv_sift: 
		# compute SIFT with OpenCV
		sift = cv2.SIFT()

		# find the keypoints and descriptors with SIFT
		list_kp1, des1 = sift.detectAndCompute(qImg,None)
		list_kp2, des2 = sift.detectAndCompute(tImg,None)
	
		kp1 = convert_keypoints_opencv_vlfeat(list_kp1);
		kp2 = convert_keypoints_opencv_vlfeat(list_kp2);

	else:
		# compute SIFT with vlfeat
		import vlfeat;

		I = np.array(qImg, dtype=np.float32);
		_kp1, _des1 = vlfeat.vl_sift(data=I);
		des1 = np.transpose(np.array(_des1, copy=True));
		kp1 = np.transpose(np.array(_kp1, copy=True));

		I = np.array(tImg, dtype=np.float32);
		_kp2, _des2 = vlfeat.vl_sift(data=I);
		des2 = np.transpose(np.array(_des2, copy=True));
		kp2 = np.transpose(np.array(_kp2, copy=True));

	if kp1.shape[0] > 0 and kp2.shape[0] > 0:
		boolidx1 = np.logical_and(kp1[:, 0] >= x1, kp1[:, 0] <= x2);
		boolidx2 = np.logical_and(kp1[:, 1] >= y1, kp1[:, 1] <= y2);
		boolidx = np.logical_and(boolidx1, boolidx2);

		des1 = des1[boolidx, :];
		kp1 = kp1[boolidx, :]

		matches = match_and_ratio_test(des1, des2);

	else:
		matches = [];

	return kp1, kp2, matches, des1, des2;


# def rescale_rect(left, top, width, height, scale):
# 	ox = left+0.5*width;
# 	oy = top+0.5*height;
# 	width *= scale;
# 	height *= scale;

# 	left = ox - 0.5*width;
# 	top = oy - 0.5*height;

# 	return (left, top, width, height);

# def pad_rect(left, top, width, height, npad):
# 	left -= npad;
# 	top -= npad;
# 	width += 2*npad
# 	height += 2*npad;

# 	return (left, top, width, height);

def pad_annorect(r, npad):
	r.x1 -= npad;
	r.y1 -= npad;
	r.x2 += npad
	r.y2 += npad;


def RectSIFT(qImg, _qRect, tImg, _tRect):

        qRect = copy.deepcopy(_qRect);
        tRect = copy.deepcopy(_tRect);

	# MA: make smaller rect relative to the second 
	if qRect.width() > 250:
		roi_scale = 0.85;
	else: 
		roi_scale = 0.95;

        qRectSmall = copy.deepcopy(qRect);
        qRectSmall.resize(roi_scale);

        # MA: pad since SIFT features seem to depend on the size of the cropped image
	npad = 20;
        pad_annorect(qRect, npad);
        pad_annorect(tRect, npad);

        qRect.clipToImage(0, qImg.shape[1] - 1, 0, qImg.shape[0] - 1);
        tRect.clipToImage(0, tImg.shape[1] - 1, 0, tImg.shape[0] - 1);

        # make small rect relative to the cropped image
        qRectSmall.x1 -= qRect.x1;
        qRectSmall.y1 -= qRect.y1;
        qRectSmall.x2 -= qRect.x1;
        qRectSmall.y2 -= qRect.y1;

	qImgTemp = qImg[int(qRect.y1):int(qRect.y2)+1, int(qRect.x1):int(qRect.x2)+1];
	tImgTemp = tImg[int(tRect.y1):int(tRect.y2)+1, int(tRect.x1):int(tRect.x2)+1];

	assert(qImgTemp.shape[0] != 0 and qImgTemp.shape[1] != 0 and tImgTemp.shape[0] != 0 and tImgTemp.shape[1] != 0);

	kp1, kp2, matches, des1, des2 = SIFT(qImgTemp, tImgTemp, qRectSmall);

	for kidx in range(kp1.shape[0]):
		kp1[kidx, 0] += qRect.x1
		kp1[kidx, 1] += qRect.y1;
		
	for kidx in range(kp2.shape[0]):
		kp2[kidx, 0] += tRect.x1;
		kp2[kidx, 1] += tRect.y1;

	return kp1, kp2, matches, des1, des2;

def outlierFiltering(X1, Y1, X2, Y2, d):
	xDis = [X2[i]-X1[i] for i in range(len(X1))];
	yDis = [Y2[i]-Y1[i] for i in range(len(Y1))];
	if (d > 0):
		Xbins = [min(xDis)+i*d for i in range(int((max(xDis)-min(xDis))/d)+2)];
		Ybins = [min(yDis)+i*d for i in range(int((max(yDis)-min(yDis))/d)+2)];
		xHist = np.histogram(xDis, Xbins);	
		yHist = np.histogram(yDis, Ybins);
        else:
		xHist = np.histogram(xDis);	
		yHist = np.histogram(yDis);
#	print xHist;
#	print yHist;	
	maxXhist = max(xHist[0])
	maxYhist = max(yHist[0])
	
	xInd = [i for i,j in enumerate(xHist[0]) if j==maxXhist];
	yInd = [i for i,j in enumerate(yHist[0]) if j==maxYhist];
	
	xInd = xInd[0]; yInd = yInd[0];
	
	ddx = xHist[1][1]-xHist[1][0]; ddy = yHist[1][1]-yHist[1][0];
	
	ddx = max(2,ddx);  ddy = max(2,ddy);
	Inds = [i for i in range(len(X1)) if 
			(xDis[i] >= xHist[1][xInd]-ddy and 
			xDis[i] <= xHist[1][xInd]+ddx and
			yDis[i] >= yHist[1][yInd]-ddy and 
			yDis[i] <= yHist[1][yInd]+ddy)]
	X1t = []; X2t = []; Y1t=[]; Y2t = [];
        for i in Inds:
		X1t.append(X1[i]);
		X2t.append(X2[i]);
		Y1t.append(Y1[i]);
		Y2t.append(Y2[i]);
	ratio = float(len(X1t))/len(X1);
	return X1t, Y1t, X2t, Y2t, ratio;
	
def R2Mapping(X1, Y1, X2, Y2):
	if (len(X1)<2):
		return False, None, None, None
        
	X1, Y1, X2, Y2, r = outlierFiltering(X1, Y1, X2, Y2, 0);

	if (len(X1)<2):
		return False, None, None, None

	X1t, Y1t, X2t, Y2t, r = outlierFiltering(X1, Y1, X2, Y2, 1);

	if (r>.8):
		X1, Y1, X2, Y2 = X1t, Y1t, X2t, Y2t;

	if (len(X1)<2):
		return False, None, None, None;

        n = len(X1);

        #print n

	A = np.zeros((2*n, 3));
	b = np.zeros((2*n, 1));

	A[0::2, 0] = X1;
	A[1::2, 0] = Y1;
	A[0::2, 1] = 1;
	A[1::2, 2] = 1;

	b[0::2, 0] = X2;
	b[1::2, 0] = Y2;

	res = np.linalg.lstsq(A, b);
	x = res[0];

	return (True, float(x[1]), float(x[2]), float(x[0]))


def ShowImg(WinName, Img, Rect = []):
	tmpImg = copy.deepcopy(Img)

	if Rect != []:
            for r in Rect:
		cv2.rectangle(tmpImg, (int(r.x1),int(r.y1)), (int(r.x2), int(r.y2)), 255, 2)

	cv2.imshow(WinName, tmpImg)
	cv2.waitKey(0)

def NextRect(Img1, Img2, Rect1):

	X_max = len(Img2[0])-1; Y_max = len(Img2)-1;
	
	clipped_rect = Rect1;
	clipped_rect.clipToImage(0, X_max, 0, Y_max);

	if clipped_rect.width() == 0 or clipped_rect.height() == 0:
		assert(False);

	kp1, kp2, matches, des1, des2 = RectSIFT(Img1, clipped_rect, Img2, clipped_rect);

	X1 = []; X2 = []; Y1=[]; Y2 = [];
        for i in range(len(matches)):
		X1.append(kp1[matches[i].queryIdx, 0]);
		Y1.append(kp1[matches[i].queryIdx, 1]);

		X2.append(kp2[matches[i].trainIdx, 0]);
		Y2.append(kp2[matches[i].trainIdx, 1]);
       
	if len(X1)<2:
		return (False, [], [], [], []);
	else: 
		flag, dx, dy, alpha = R2Mapping(X1, Y1, X2, Y2);

		if not flag:
			return (False, [], [], [], []);
		
		new_rect = AnnoRect(dx + alpha*clipped_rect.x1, dy + alpha*clipped_rect.y1, dx + alpha*clipped_rect.x2, dy + alpha*clipped_rect.y2);
		new_rect.clipToImage(0.0, X_max, 0.0, Y_max);

		return (True, new_rect, matches, des1, des2);

def track_frame(a, stop_imgname, trackMaxFrames, frame_inc):

	MIN_TRACK_RECT_SIZE = 30;
	max_move_thr = .9;
	#min_match_percentage = 0.25;
	#min_match_percentage = 0.05;
	#max_color_dist = 1.7;
        #max_color_dist = 0.25;
        max_color_dist = 0.3;

	min_match_percentage = 0;

	annolist_track = [];

        framesTracked = 0;
        num_missed_tracks = 0;

        num_init_matching_failed_color = 0;
        num_init_matching_failed_sift = 0;
        num_ignore_border = 0;

        num_skip_small = 0;

        curImageName = a.imageName;

	# filter initial set of rectangles 
	tracked_rects = [r for r in a.rects if r.width() > MIN_TRACK_RECT_SIZE and r.height() > MIN_TRACK_RECT_SIZE];

	# don't include heavily occluded 
        tracked_rects = filter_occluded(tracked_rects);

        # MA: store descriptors and number of matches for track verification
        tracks_init_des = []
        tracks_init_num_matches = [];
	tracks_init_colorhist = [];

        Img1 = cv2.imread(curImageName, 0);
	Img1_color = cv2.imread(curImageName);

        # MA: init track id's
        for tidx, r in enumerate(tracked_rects):
	    if frame_inc > 0:
		    r.classID = tidx;
	    else:
		    r.classID = -tidx;
			
            tracks_init_des.append([]);
            tracks_init_num_matches.append(-1);

            print "width: ", r.width(), ", height: ", r.height();

	    assert(r.width > 0 and r.height() > 0);
	    tracks_init_colorhist.append(comp_rect_hist(Img1_color, r));

        if not isinstance(Img1, np.ndarray):
            assert(False);

        print "curImageName: " + curImageName

        while curImageName != stop_imgname:

            if not os.path.isfile(curImageName):
                print "image does not exist: " + curImageName
                assert(False);

            a = Annotation();
            a.imageName = curImageName;
            a.rects = tracked_rects;

            annolist_track.append(a);

            # track to next image
            nextImageName = inc_image_name(curImageName, frame_inc);
            print "\tnextImageName: " + nextImageName

            if os.path.isfile(nextImageName):

                Img2 = cv2.imread(nextImageName, 0);
                Img2_color = cv2.imread(nextImageName);

                assert(isinstance(Img2, np.ndarray))
                img_height = len(Img2)
                img_width = len(Img2[0]);

                assert(img_height > 0 and img_width > 0);
                new_tracked_rects = [];

                # tracking 
                for rect in tracked_rects:
                    if rect.width() <= MIN_TRACK_RECT_SIZE or rect.height() <= MIN_TRACK_RECT_SIZE:
                        num_skip_small += 1;
                        continue;

                    track_ok, new_rect, matches, des1, des2 = NextRect(Img1, Img2, rect);

                    if track_ok and new_rect.width() > 1 and new_rect.height() > 1:
                        # preserve classID
                        new_rect.classID = rect.classID;

                        cur_num_matches = 0;

                        if framesTracked == 0:
                            tracks_init_des[abs(rect.classID)] = des1;
                            tracks_init_num_matches[abs(rect.classID)] = len(matches);

                            cur_num_matches = tracks_init_num_matches[abs(rect.classID)];
                        else:
                            bf = cv2.BFMatcher();
                            cur_num_matches = len(match_and_ratio_test(tracks_init_des[abs(rect.classID)], des2));
                        
                        print "\ttrack %d, init_num_matches %d, cur_num_matches: %d" % (rect.classID, tracks_init_num_matches[abs(rect.classID)], cur_num_matches)

			verification_ok = True;

			# MA: color-based verification
		        hist_cur = comp_rect_hist(Img2_color, new_rect);
			
			#cur_color_dist = cv2.compareHist(tracks_init_colorhist[rect.classID], hist_cur, cv.CV_COMP_CHISQR)
			cur_color_dist = dist_chi2(tracks_init_colorhist[abs(rect.classID)], hist_cur);

			print "\n\t cur_color_dist: " + str(cur_color_dist);

			if  cur_color_dist > max_color_dist and new_rect.overlap_pascal(rect) < max_move_thr:
                            verification_ok = False;
                            num_init_matching_failed_color += 1;

			# MA: SIFT-based verification (seems to be too conservative) 
                        # if cur_num_matches < min_match_percentage*tracks_init_num_matches[rect.classID]:
			# 	verification_ok = False;
                        #         num_init_matching_failed_sift += 1;

                        # MA: for now tracks that start on the boundary should remain on the boundary (otherwise we don't know the correct extent)
                        BORDER_THRESHOLD = 10;
			car_dim_ratio = 1.5
			if rect.height() > .3 * Img2.shape[0]:
				car_dim_ratio = 1.2
			if(rect.x1 < BORDER_THRESHOLD and new_rect.x1 > rect.x1):
			   if(new_rect.height()<50):
				new_rect.y1 -= 10;
			   if (1.0 * new_rect.width()/new_rect.height()) < car_dim_ratio:
				new_rect.x1 = 0
		
			if(rect.x2 > Img2.shape[1] - BORDER_THRESHOLD and new_rect.x2 < rect.x2):
			   if(new_rect.height()<50):
				new_rect.y1 -= 10;
			   if (1.0 * new_rect.width()/new_rect.height()) < car_dim_ratio:
				new_rect.x2 = Img2.shape[1]-1;
                             
			# MA: accept first frame (use it as baseline number of matches for SIFT)
                        if verification_ok:
                            new_tracked_rects.append(new_rect);

                    else:
                        num_missed_tracks += 1;

                print "\tnum_active_tracks: %d, num_missed_tracks: %d, num_skip_small: %d, num_init_matching_failed_sift: %d, num_init_matching_failed_color: %d, num_ignore_border: %d\n" \
                    % (len(new_tracked_rects), num_missed_tracks, num_skip_small, num_init_matching_failed_sift, num_init_matching_failed_color, num_ignore_border)

                framesTracked += 1;
            
                if framesTracked >= trackMaxFrames: 
                    break;

                curImageName = nextImageName;
        	tracked_rects = filter_occluded(new_tracked_rects);
                Img1 = Img2;

            else:
                print "End of sequence, could not find: " + nextImageName
                break;
        
        
	return annolist_track;
