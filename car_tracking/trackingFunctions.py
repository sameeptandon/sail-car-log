import numpy as np
import cv2
from matplotlib import pyplot as plt
import math
import sys

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
def SIFT(qImg, tImg, rect):

        # Initiate SIFT detector
        sift = cv2.SIFT()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(qImg,None)
        kp2, des2 = sift.detectAndCompute(tImg,None)

	
	# MA: test that keypoints are inside the region of interest rect
	x1 = rect[0];
	y1 = rect[1];
	x2 = x1 + rect[2];
	y2 = y1 + rect[3];

	X1 = [(idx1, k) for idx1, k in enumerate(kp1) if k.pt[0] >= x1 and k.pt[0] <= x2 and k.pt[1] >= y1 and k.pt[1] <= y2];
	new_kp1_idx, new_kp1 = zip(*X1);

	#X2 = [(idx1, k) for idx1, k in enumerate(kp2) if k.pt[0] >= x1 and k.pt[0] <= x2 and k.pt[1] >= y1 and k.pt[1] <= y2];
	#new_kp2_idx, new_kp2 = zip(*X2);
	
	des1 = des1[list(new_kp1_idx)];
	#des2 = des2[list(new_kp2_idx)];

	kp1 = list(new_kp1);
	#kp2 = list(new_kp2);

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

		#pdb.set_trace();
		
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

	# qx, qy, qw, qh = qRect;
	# tx, ty, tw, th = tRect;
	
	# qx, qy, qw, qh = rescale_rect(qx, qy, qw, qh, 1.0);
	# tx, ty, tw, th = rescale_rect(tx, ty, tw, th, 1.0);
	
	# qx1, qy1, qw1, qh1 = rescale_rect(qx, qy, qw, qh, 0.75);

	# qx2, qy2, qw2, qh2 = pad_rect(qx, qy, qw, qh, 100.0);
	# tx2, ty2, tw2, th2 = pad_rect(tx, ty, tw, th, 100.0);

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

	for k in kp1:
		x, y = k.pt;
		k.pt = (x + qx2, y + qy2);
		
	for k in kp2:
		x, y = k.pt;
		k.pt = (x + tx2, y + ty2);


	# MA: output is exepected to be in the reference frame of the input rectangle
	# for k in kp1:
	# 	x, y = k.pt;
	# 	#k.pt = (x + qx2, y + qy2);
	# 	k.pt = (x - npad, y - npad);

	# for k in kp2:
	# 	x, y = k.pt;
	# 	#k.pt = (x + tx2, y + ty2);
	# 	k.pt = (x - npad, y - npad);

	# DEBUG - begin
	# for i in range(len(matches)):
	# 	x1, y1 =  kp1[matches[i].queryIdx].pt;
	# 	x2, y2 = kp2[matches[i].trainIdx].pt;
	# 	x1 = int(x1); x2 = int(x2); y1 = int(y1); y2 = int(y2);
	# 	cv2.circle(qImg, (x1,y1), 2, 255);
	# 	cv2.circle(tImg, (x2,y2), 2, 255);

	# cv2.imshow('img1', qImg);
	# cv2.imshow('img2', tImg);

	# cv2.waitKey(0);
	#pdb.set_trace();
	# DEBUG - end

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

	# A_io = sum([float(X1[i])*X2[i] + Y1[i]*Y2[i] for i in range(n)])
	# A_ii = sum([float(X1[i])*X1[i] + Y1[i]*Y1[i] for i in range(n)])
	# A_i = [sum(X1), sum(Y1)];
	# A_o = [sum(X2), sum(Y2)];
	# al_nom = (A_io - (A_i[0]*A_o[0]+A_i[1]*A_o[1])/n);
	# al_denom = (A_ii - (A_i[0]*A_i[0]+A_i[1]*A_i[1])/n);
	# if (al_denom==0):
	# 	return False, Mu, alpha
	# alpha = al_nom/al_denom;
        # Mu= [(A_o[0]-alpha*A_i[0])/n,(A_o[1]-alpha*A_i[1])/n]
        # return True, Mu, alpha

def ShowImg(WinName, Img, Rect):
	ImgTemp = Img;
	x,y,w,h = Rect;
	ImgTemp2 = cv2.rectangle(Img, (int(x),int(y)), (int(x+w), int(y+h)), 255, 2);
	cv2.imshow(WinName, Img);
	cv2.waitKey(0)


def NextRect(Img1, Img2, Rect1):

	X_max = len(Img2[0])-1; Y_max = len(Img2)-1;

	# dx = 0; dy = 0;
	# x1, y1, w1, h1 = Rect1;
	# x2 = max(x1-dx,0);
        # y2 = max(y1-dy,0);
        # w2 = min(X_max, x1+w1+dx) - x2;
        # h2 = min(Y_max, y1+h1+dy) - y2;

	# Rect2 = (x2, y2, w2, h2);
	
	clipped_rect = Rect1;
	clipped_rect.clipToImage(0, X_max, 0, Y_max);

	if clipped_rect.width() == 0 or clipped_rect.height() == 0:
		assert(False);

	kp1, kp2, matches = RectSIFT(Img1, clipped_rect, Img2, clipped_rect);

	X1 = []; X2 = []; Y1=[]; Y2 = [];
        for i in range(len(matches)):
                keypoint1 = kp1[matches[i].queryIdx];
                keypoint2 = kp2[matches[i].trainIdx];
                X1.append(keypoint1.pt[0]);
                Y1.append(keypoint1.pt[1]);
                X2.append(keypoint2.pt[0]);
                Y2.append(keypoint2.pt[1]);
       
	if len(X1)<2:
		#return (False, Rect1);
		return (False, []);
	else: 
		flag, Mu, alpha = R2Mapping(X1, Y1, X2, Y2);

		if not flag:
			#return (False, Rect1);
			return (False, []);

		x_cur, y_cur, w_cur, h_cur = (clipped_rect.x1, clipped_rect.y1, clipped_rect.width(), clipped_rect.height());
		
		# x_new = Mu[0] + x_cur;
		# y_new = Mu[1] + y_cur;

		x_new = round(Mu[0] + alpha*x_cur);
		y_new = round(Mu[1] + alpha*y_cur);

		w_new = round(alpha*w_cur);
		h_new = round(alpha*h_cur);


		new_rect = AnnoRect(x_new, y_new, x_new + w_new, y_new + h_new);
		# r.x1 = x_new;
		# r.y1 = y_new;
		# r.x2 = x_new + w_new;
		# r.y2 = y_new + h_new;
		
		new_rect.clipToImage(0, X_max, 0, Y_max);

		#pdb.set_trace();		

		return (True, new_rect);

		# return (True, (max((x_new),0), max((y_new),0), 
		# 		min((w_new),X_max-x_new), min((h_new),Y_max-y_new)))
	
