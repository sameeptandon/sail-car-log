import numpy as np
import cv2
from matplotlib import pyplot as plt
import math
import sys


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
def SIFT(qImg, tImg):

        # Initiate SIFT detector
        sift = cv2.SIFT()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(qImg,None)
        kp2, des2 = sift.detectAndCompute(tImg,None)

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
		
	#	for i in range(len(matches)):
	#                x1,y1 =  kp1[matches[i].queryIdx].pt;
	#                x2, y2 = kp2[matches[i].trainIdx].pt;
	#                x1 = int(x1); x2 = int(x2); y1 = int(y1); y2 = int(y2);
	#                cv2.circle(qImg, (x1,y1), 2, 255);
	#                cv2.circle(tImg, (x2,y2), 2, 255);

		return kp1, kp2, matches;
	except:
		return kp1, kp2, [];


def RectSIFT(qImg, qRect, tImg, tRect):
	qx, qy, qw, qh = qRect;
	tx, ty, tw, th = tRect;

	qImgTemp = qImg[int(qy):int(qy+qh), int(qx):int(qx+qw)];
	tImgTemp = tImg[int(ty):int(ty+th), int(tx):int(tx+tw)];

	if qImgTemp.shape[0] == 0 or qImgTemp.shape[0] == 0 or tImgTemp.shape[0] == 0 or tImgTemp.shape[0] == 0:
		assert(False);
	
	return SIFT(qImgTemp, tImgTemp);

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

	A_io = sum([float(X1[i])*X2[i] + Y1[i]*Y2[i] for i in range(n)])
	A_ii = sum([float(X1[i])*X1[i] + Y1[i]*Y1[i] for i in range(n)])
	A_i = [sum(X1), sum(Y1)];
	A_o = [sum(X2), sum(Y2)];
	al_nom = (A_io - (A_i[0]*A_o[0]+A_i[1]*A_o[1])/n);
	al_denom = (A_ii - (A_i[0]*A_i[0]+A_i[1]*A_i[1])/n);
	if (al_denom==0):
		return False, Mu, alpha
	alpha = al_nom/al_denom;
        Mu= [(A_o[0]-alpha*A_i[0])/n,(A_o[1]-alpha*A_i[1])/n]
        return True, Mu, alpha

def ShowImg(WinName, Img, Rect):
	ImgTemp = Img;
	x,y,w,h = Rect;
	ImgTemp2 = cv2.rectangle(Img, (int(x),int(y)), (int(x+w), int(y+h)), 255, 2);
	cv2.imshow(WinName, Img);
	cv2.waitKey(0)

def NextRect(Img1, Img2, Rect1):
	dx = 0; dy = 0;

	X_max = len(Img2[0])-1; Y_max = len(Img2)-1;
	x1, y1, w1, h1 = Rect1;
	x2 = max(x1-dx,0);
        y2 = max(y1-dy,0);
        w2 = min(X_max, x1+w1+dx) - x2;
        h2 = min(Y_max, y1+h1+dy) - y2;
	Rect2 = (x2, y2, w2, h2);

	if w2 == 0 or h2 == 0:
		assert(False);

	kp1, kp2, matches = RectSIFT(Img1, Rect1, Img2, Rect2);
	X1 = []; X2 = []; Y1=[]; Y2 = [];
        for i in range(len(matches)):
                keypoint1 = kp1[matches[i].queryIdx];
                keypoint2 = kp2[matches[i].trainIdx];
                X1.append(keypoint1.pt[0]);
                Y1.append(keypoint1.pt[1]);
                X2.append(keypoint2.pt[0]);
                Y2.append(keypoint2.pt[1]);
       
	if len(X1)<2:
		return (False, Rect1);
	else: 
		flag, Mu, alpha = R2Mapping(X1, Y1, X2, Y2);
		if not flag:
			return (False, Rect1);
		x_new = Mu[0] + x2;
		y_new = Mu[1] + y2;
		w_new = alpha*w1;
		h_new = alpha*h1;
		return (True, (max((x_new),0), max((y_new),0), 
				min((w_new),X_max-x_new), min((h_new),Y_max-y_new)))
	
