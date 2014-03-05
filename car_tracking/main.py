import sys
import numpy as np
import cv2
import cv
from trakingFunctions import *
from trakingFunctions import NextRect
from AnnotationLib import *


def ImageNames(ImgName,n):
	ImgNames = [];
	p = ImgName.split(".");
	p0 = ".".join(p[:-1]);
	p00 = '_'.join(p0.split("_")[:-1]);
	ImgNum = int(p[-2].split("_")[-1]);
	for i in range(n):
		IN = p00 + "_" + "%04d"%(ImgNum+i+1) + ".jpeg";
		ImgNames.append(IN);
	return ImgNames;

if __name__ == "__main__":
	if (len(sys.argv) < 2):
		print "Please Enter The Frame#"
		sys.exit();
	index = int(sys.argv[1]);
	annotations = parseXML('../Data/7-16-sacramento.al');
	annotation = annotations[index];
	ImgName = "../Data/" + annotation.filename();
	

	Img1 = cv2.imread(ImgName,0);
	cv2.imshow('img1',Img1);
	cv2.waitKey(0);

	rects = annotation.rects;
	tempImg1 = cv2.imread(ImgName,0);
	for rect in rects:
		tempImgg1 = cv2.rectangle(tempImg1, (int(rect.x1),int(rect.y1)), (int(rect.x2), int(rect.y2)), 255,2);

		
	cv2.imshow('img1',tempImg1);
	cv2.waitKey(0);
	winFlags = [True for i in range(len(rects))];
	
	ImgNames = ImageNames(ImgName,50);
	flag = 1;
	i = 0;
	
#	rect = rects[0];
	windows = [(rect.x1, rect.y1, rect.x2-rect.x1, rect.y2-rect.y1) for rect in rects];
	newWindows = windows;
	n_win = len(windows)
	while(1):
		print i
		Img2 = cv2.imread(ImgNames[i],0);
		for j in range(n_win):
			if winFlags[j]:
				win = windows[j];
				try:
					NextWinInfo = NextRect(Img1, Img2, win);
					print NextWinInfo
					if not NextWinInfo[0]:
						winFlags[j] = False;
					else:
						newWindows[j] = NextWinInfo[1];
				except:
					winFlags[j] = False;

		print winFlags;
		tempImg2 = Img2;
		t = False;
		for j in range(n_win):
			if winFlags[j]:
				newWin = newWindows[j];
				t = True;
				tempImgg2 = cv2.rectangle(tempImg2, 
						(int(newWin[0]),int(newWin[1])), 
						(int(newWin[0] + newWin[2]), 
						int(newWin[1]+newWin[3])), 255,2);	
				
		Img1 = cv2.imread(ImgNames[i],0);
		windows = newWindows;		
		i +=1;
		if i > 49 or not t:
			break;
		cv2.imshow('img1',tempImg2);
		cv2.waitKey(60); 
		
