#implements the mapping system
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

#given as input a lambda, the function outputs its best prediction for the
#location of the point.
def _predict(lam):
	m = data.shape[0];
	W = np.zeros([m,m]);
	tau = 0.9;
	for i in range(0, m):
		W[i,i] = np.exp(-((lam - datalambs[i, 1])**2)/(2*(tau**2)));
		#intermediary result to allow calc of both X and Y without redundancy
		inter1 = np.linalg.pinv(reduce(np.dot, [datalambs.T, W, datalambs]))
		inter = reduce(np.dot, [inter1, datalambs.T, W]);
	[thetaX, thetaY] = inter.dot(data[:, 0]), inter.dot(data[:,1]);
	[estimateX, estimateY] = [thetaX[1]*lam + thetaX[0], thetaY[1]*lam + thetaY[0]];
	return [estimateX, estimateY];

#this is the objective function we minimize over. It is the sum of squares of
#x difference and y difference
def _minf(lam, x, y):
	[estimateX, estimateY] = _predict(lam)
	return (x - estimateX)**2 + (y - estimateY)**2;

#call predict on a list of lambdas, returning an array of coordinates
def generateMap(lambdas):
	predtest = np.zeros([lambdas.size, 2]);
	for i in range(0, lambdas.size):
		predtest[i] = _predict(lambdas[i])
	return predtest;

#called when newData is to be added to the map
def addToMap(newcoords):
	global data, datalambs
	for newcoord in newcoords:
		minimized = minimize(_minf, 0, (newcoord));
		indexToAdd = int(np.ceil(minimized.x[0]));
		data = np.insert(data, indexToAdd, newcoord, axis =0);
		datalambs = np.vstack((datalambs,[1.,data.shape[0]-1]))

#useful for scatterplots of coordinate data
def draw(data, color):
	plt.scatter(data[:,0], data[:,1], color= color);

#getting sample lambdas
def getSampleLambdas():
	return np.arange(0., data.shape[0]+1, 0.1);


#data - original and new
data = np.array([[5,1], [4.5,2], [3.75, 2.5], [3,3], [2.3, 3.4], [2, 4], [1.7, 5], [1.8,6], [2.4, 7], [3,8]]);
newData = np.array([[2.5, 2.6], [2.5, 7.5], [4.0, 2.4]]);
datalambs = np.vstack((np.ones(data.shape[0]), np.arange(data.shape[0]))).T

#draw the original data
draw(data, "blue");

predCoords = generateMap(getSampleLambdas());
addToMap(newData);
newPredCoords = generateMap(getSampleLambdas());


draw(newData, "black");
draw(predCoords, "red");
draw(newPredCoords, "green");

plt.show();
