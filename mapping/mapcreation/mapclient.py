"""Implements a tester for the mapping module."""

import matplotlib.pyplot as plt
from mapper import Map
import numpy as np
import sys

#useful for scatterplots of coordinate data
def draw(data, color):
	plt.scatter(data[:,0], data[:,1], color= color);

#run the mapper with the original data and new Map
def run(data, newData):
	m = Map(data);
	predCoords = m.generateMap();
	#draw the original data
	#draw(data, "blue");
	m.addToMap(newData);
	newPredCoords = m.generateMap();
	#draw(newData, "black");
	#draw(predCoords, "red");
	draw(data, "black");
	draw(newPredCoords, "green");
	plt.show();

#run a toy simulation
def toyRun():
	data = np.array([[5,1], [4.5,2], [3.75, 2.5], [2, 4], [1.7, 5], [1.8,6], [2.4, 7], [3,8]]);
	newData = np.array([[3,3], [2.3, 3.4],[2.5, 2.6], [2.5, 7.5], [4.0, 2.4]]);
	run(data, newData);

#load an actualy file
def realRun(filename):
	C = np.loadtxt(open(filename, "rb"), delimiter = ',');
	C = C[0:-1:400]
	num = C.shape[0]
	startindex = int((num/2)-(num/8))
	endindex = int((num/2)+(num/8))
	newData = C[startindex:endindex, :]
	data = np.vstack((C[0:startindex,:], C[startindex:endindex:10,:], C[endindex:C.shape[0], :]))
	run(data, newData);

if __name__ == '__main__':
	realRun("101ex");
	"""if len(sys.argv)==1:
		toyRun();
	else:
		realRun(sys.argv[1]);
	"""
