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
	
def track(annotations,i,l):
#	if (len(sys.argv) < 2):
#		print "Please Enter The Frame#"
#		sys.exit();
	index = i;
#	annotations = parseXML('../Data/7-16-sacramento.al');
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

	ImgNames = ImageNames(ImgName,50);
	flag = 1;
	i = 0;
	rect = rects[l];
	window = (rect.x1, rect.y1, rect.x2-rect.x1, rect.y2-rect.y1);
	while(flag):
		Img2 = cv2.imread(ImgNames[i],0);
		NextWinInfo = NextRect(Img1, Img2, window);
		newWin = NextWinInfo[1];
		flag = NextWinInfo[0] > 0 and i<48;
		
		tempImg2 = Img2;
		tempImgg2 = cv2.rectangle(tempImg2, (int(newWin[0]),int(newWin[1])), (int(newWin[0] + newWin[2]), int(newWin[1]+newWin[3])), 255,2);	
		cv2.imshow('img3',tempImg2);
		cv2.waitKey(0);
		
		window = newWin;
		Img1 = cv2.imread(ImgNames[i],0);
#		print window;
#		print newWin;
		i = i+1;	
