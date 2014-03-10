import numpy as np
import cv2
from matplotlib import pyplot as plt
import math
import sys

import vlfeat;

from AnnotationLib import AnnoRect;

import pdb;

##########################################
# Inputs:  			       	 #
#	qImg: Query Image		 #
#	tImg: Training Image		 #
# Outputs: 				 #
#	kp1: Keypoints of query image	 #
#	kp2: Keypoints of training image #
#	matches: Matches found by SIFT	 #
#		 algorithm between kp1 	 #
#		 and kp2		 #
##########################################

def convert_keypoints_opencv_vlfeat(list_kp):
	n = len(list_kp)
	
	kp = np.ndarray(shape=(n, 4), dtype=np.float32);

	for idx, k in enumerate(list_kp):
		kp[idx, 0] = k.pt[0];
		kp[idx, 1] = k.pt[1];
		kp[idx, 2] = k.size;
		kp[idx, 3] = k.angle;

	return kp;

def SIFT(qImg, tImg, rect):

	# MA: test that keypoints are inside the region of interest rect
	x1 = rect[0];
	y1 = rect[1];
	x2 = x1 + rect[2];
	y2 = y1 + rect[3];

	use_opencv_sift = True;

	if use_opencv_sift: 
		# Initiate SIFT detector
		sift = cv2.SIFT()

		# find the keypoints and descriptors with SIFT
		list_kp1, des1 = sift.detectAndCompute(qImg,None)
		list_kp2, des2 = sift.detectAndCompute(tImg,None)
	
		kp1 = convert_keypoints_opencv_vlfeat(list_kp1);
		kp2 = convert_keypoints_opencv_vlfeat(list_kp2);

	else:
		# use vlfeat
		I = np.array(qImg, dtype=np.float32);
		_kp1, _des1 = vlfeat.vl_sift(data=I);
		des1 = np.transpose(np.array(_des1, copy=True));
		kp1 = np.transpose(np.array(_kp1, copy=True));

		I = np.array(tImg, dtype=np.float32);
		_kp2, _des2 = vlfeat.vl_sift(data=I);
		des2 = np.transpose(np.array(_des2, copy=True));
		kp2 = np.transpose(np.array(_kp2, copy=True));

	boolidx1 = np.logical_and(kp1[:, 0] >= x1, kp1[:, 0] <= x2);
	boolidx2 = np.logical_and(kp1[:, 1] >= y1, kp1[:, 1] <= y2);
	boolidx = np.logical_and(boolidx1, boolidx2);

	des1 = des1[boolidx, :];
	kp1 = kp1[boolidx, :]

	print "points in rect 1: " + str(len(kp1))
	print "points in rect 2: " + str(len(kp2)) + "\n"

        # BFMatcher with default params
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1,des2, k=2)

        # Apply ratio test
        good = []
	if len(matches)<2:
		return kp1, kp2, [];
	try:
		for m,n in matches:
		    if m.distance < 0.75*n.distance:
			good.append([m])

		matches = [good[i][0] for i in range(len(good))]		
		
		return kp1, kp2, matches;
	except:
		return kp1, kp2, [];

def rescale_rect(left, top, width, height, scale):
	ox = left+0.5*width;
	oy = top+0.5*height;
	width *= scale;
	height *= scale;

	left = ox - 0.5*width;
	top = oy - 0.5*height;

	return (left, top, width, height);

def pad_rect(left, top, width, height, npad):
	left -= npad;
	top -= npad;
	width += 2*npad
	height += 2*npad;

	return (left, top, width, height);


def RectSIFT(qImg, qRect, tImg, tRect):
	
	qx, qy, qw, qh = (qRect.x1, qRect.y1, qRect.width(), qRect.height());
	tx, ty, tw, th = (tRect.x1, tRect.y1, tRect.width(), tRect.height());


	roi_scale = 0.85;
	qx1, qy1, qw1, qh1 = rescale_rect(qx, qy, qw, qh, roi_scale);

	npad = 100;
	qx2, qy2, qw2, qh2 = pad_rect(qx, qy, qw, qh, npad);
	tx2, ty2, tw2, th2 = pad_rect(tx, ty, tw, th, npad);

	# MA: make smaller rect relative to the second 
	qx1 -= qx2;
	qy1 -= qy2;

	qImgTemp = qImg[int(qy2):int(qy2+qh2), int(qx2):int(qx2+qw2)];
	tImgTemp = tImg[int(ty2):int(ty2+th2), int(tx2):int(tx2+tw2)];

	if qImgTemp.shape[0] == 0 or qImgTemp.shape[0] == 0 or tImgTemp.shape[0] == 0 or tImgTemp.shape[0] == 0:
		return [], [], []
                #assert(False);
	
	#return SIFT(qImgTemp, tImgTemp);
	kp1, kp2, matches = SIFT(qImgTemp, tImgTemp, (qx1, qy1, qw1, qh1));

	for kidx in range(kp1.shape[0]):
		kp1[kidx, 0] += qx2
		kp1[kidx, 1] += qy2;
		
	for kidx in range(kp2.shape[0]):
		kp2[kidx, 0] += tx2;
		kp2[kidx, 1] += ty2;

	return kp1, kp2, matches;


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
	flag = True;
	Mu = [0,0];
	alpha = 1;
	if (len(X1)<2):
		return False, Mu, alpha;
	X1, Y1, X2, Y2, r = outlierFiltering(X1, Y1, X2, Y2, 0);
	if (len(X1)<2):
		return False, Mu, alpha;
	X1t, Y1t, X2t, Y2t, r = outlierFiltering(X1, Y1, X2, Y2, 1);
	if (r>.8):
		X1, Y1, X2, Y2 = X1t, Y1t, X2t, Y2t;

	if (len(X1)<2):
		return False, Mu, alpha;
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

	return (True, x[1:], x[0])


def ShowImg(WinName, Img, Rect):
	ImgTemp = Img;
	x,y,w,h = Rect;
	ImgTemp2 = cv2.rectangle(Img, (int(x),int(y)), (int(x+w), int(y+h)), 255, 2);
	cv2.imshow(WinName, Img);
	cv2.waitKey(0)


def NextRect(Img1, Img2, Rect1):

	X_max = len(Img2[0])-1; Y_max = len(Img2)-1;
	
	clipped_rect = Rect1;
	clipped_rect.clipToImage(0, X_max, 0, Y_max);

	if clipped_rect.width() == 0 or clipped_rect.height() == 0:
		assert(False);

	kp1, kp2, matches = RectSIFT(Img1, clipped_rect, Img2, clipped_rect);

	X1 = []; X2 = []; Y1=[]; Y2 = [];
        for i in range(len(matches)):
		X1.append(kp1[matches[i].queryIdx, 0]);
		Y1.append(kp1[matches[i].queryIdx, 1]);

		X2.append(kp2[matches[i].trainIdx, 0]);
		Y2.append(kp2[matches[i].trainIdx, 1]);
       
	if len(X1)<2:
		return (False, []);
	else: 
		flag, Mu, alpha = R2Mapping(X1, Y1, X2, Y2);

		if not flag:
			return (False, []);

		x_cur, y_cur, w_cur, h_cur = (clipped_rect.x1, clipped_rect.y1, clipped_rect.width(), clipped_rect.height());

		# MA: convert to float, otherwise x_new is an array -> problems later
		x_new = float(Mu[0] + alpha*x_cur);
		y_new = float(Mu[1] + alpha*y_cur);

		w_new = float(alpha*w_cur);
		h_new = float(alpha*h_cur);

		new_rect = AnnoRect(x_new, y_new, x_new + w_new, y_new + h_new);
		
		new_rect.clipToImage(0.0, X_max, 0.0, Y_max);

		return (True, new_rect);

