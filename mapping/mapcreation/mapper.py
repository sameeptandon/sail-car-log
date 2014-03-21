#implements the mapping system
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

class Map(object):
	def __init__(self, data):
		self.data = data;
		self.datalambs = np.vstack((np.ones(data.shape[0]), np.arange(data.shape[0]))).T;

	#getting sample lambdas
	def getSampleLambdas(self):
		return np.linspace(0., self.data.shape[0], 80);

	#given as input a lambda, the function outputs its best prediction for the
	#location of the point.
	def predict(self,lam):
		m = self.data.shape[0];
		W = np.zeros([m,m]);
		tau = 0.1;
		for i in range(0, m):
			W[i,i] = np.exp(-((lam - self.datalambs[i, 1])**2)/(2*(tau**2)));
			#intermediary result to allow calc of both X and Y without redundancy
			inter1 = np.linalg.pinv(reduce(np.dot, [self.datalambs.T, W, self.datalambs]))
			inter = reduce(np.dot, [inter1, self.datalambs.T, W]);
		[thetaX, thetaY] = inter.dot(self.data[:, 0]), inter.dot(self.data[:,1]);
		[estimateX, estimateY] = [thetaX[1]*lam + thetaX[0], thetaY[1]*lam + thetaY[0]];
		return [estimateX, estimateY];

	#this is the objective function we minimize over. It is the sum of squares of
	#x difference and y difference
	def minf(self,lam, x, y):
		[estimateX, estimateY] = self.predict(lam)
		return (x - estimateX)**2 + (y - estimateY)**2;

	#call predict on a list of lambdas, returning an array of coordinates
	def generateMap(self):
		lambdas = self.getSampleLambdas();
		predtest = np.zeros([lambdas.shape[0], 2]);
		for i in range(0, lambdas.shape[0]):
			predtest[i] = self.predict(lambdas[i])
		return predtest;

	#called when newData is to be added to the map
	def addToMap(self,newcoords):
		for newcoord in newcoords:
			minimized = minimize(self.minf, 0, (newcoord));
			indexToAdd = int(np.ceil(minimized.x[0]));
			self.data = np.insert(self.data, indexToAdd, newcoord, axis =0);
			self.datalambs = np.vstack((self.datalambs,[1.,self.data.shape[0]-1]))
