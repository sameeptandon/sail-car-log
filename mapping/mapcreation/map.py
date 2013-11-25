#implements the mapping system
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

#data - original and new
data = np.array([[5,1], [4.5,2], [3.75, 2.5], [3,3], [2.3, 3.4], [2, 4], [1.7, 5], [1.8,6], [2.4, 7], [3,8]]);
newData = np.array([[2.5, 3.1], [2.5, 7.5], [4.0, 2.4]]);

#we parameterize by lambdas
def getLambdas(data):
	ones = np.ones(data.shape[0]);
	return np.vstack((ones, np.arange(data.shape[0]))).T

#given as input a lambda, the function outputs its best prediction for the
#location of the point.
def predict(lam, datalambdas):
	m = data.shape[0];
	W = np.zeros([m,m]);
	tau = 0.5;
	for i in range(0, m):
		W[i,i] = np.exp(-((lam - datalambdas[i, 1])**2)/(2*(tau**2)));
		#intermediary result to allow calc of both X and Y without redundancy
		inter =	np.linalg.pinv(datalambdas.T.dot(W).dot(datalambdas)).dot(datalambdas.T).dot(W);
	[thetaX, thetaY] = inter.dot(data[:, 0]), inter.dot(data[:,1]);
	[estimateX, estimateY] = [thetaX[1]*lam + thetaX[0], thetaY[1]*lam + thetaY[0]];
	return [estimateX, estimateY];

#this is the objective function we minimize over. It is the sum of squares of
#x difference and y difference
def minf(lam, coord, datalambdas):
	[estimateX, estimateY] = predict(lam, datalambdas)
	return (coord[0] - estimateX)**2 + (coord[1] - estimateY)**2;

#call predict on a list of lambdas, returning an array of coordinates
def fit(lambdas):
	predtest = np.zeros([lambdas.size, 2]);
	datalambs = getLambdas(data)
	for i in range(0, lambdas.size):
		predtest[i] = predict(lambdas[i], datalambs)
	return predtest;

def addToMap(newcoords):
	for newcoord in newcoords:
		minimized = minimize(minf, 0, (newcoord, getLambdas(data)));
		indexToAdd = np.ceil(minimized.x[0]);
		data = np.insert(data, indexToAdd, newcoord, axis =0);

def drawMap(lambdas, color):
	predCoords = fit(lambdas);
	plt.scatter(predCoords[:0], predCoords[:,1], color= color);

#graph original data points
plt.scatter(data[:,0], data[:,1]);

#checking fit on original data points
test = np.linspace(0., 10., 80)
predCoords = fit(test)
plt.scatter(predCoords[:,0], predCoords[:,1], color = "r");
addToMap(newData);
plt.show();
