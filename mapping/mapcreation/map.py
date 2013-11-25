#implements the mapping system
import numpy as np
import matplotlib.pyplot as plt

Data = np.array([[5,1], [4.5,2], [3.75, 2.5], [3,3], [2.3, 3.4], [2, 4], [1.7, 5], [1.8,6], [2.4, 7], [3,8]]);
X = Data[:,0];
Y = Data[:,1];
m = X.size;
assert m == Y.size
a = np.ones(m);
index = np.arange(X.size);
b = np.vstack((a, index)).T
tau = 0.5;
lins = np.linspace(0., 10., 80)
outputX = np.zeros(lins.size);
outputY = np.zeros(lins.size);
W = np.zeros([m, m]);
for i in range(0, lins.size):
	for l in range(0, m):
		W[l,l] = np.exp(-np.power(lins[i] - b[l, 1],2)/(2*(tau**2)));
	inter = np.linalg.pinv(b.T.dot(W).dot(b)).dot(b.T).dot(W);
	thetaX = inter.dot(X);
	thetaY = inter.dot(Y);
	outputX[i] = thetaX[1]*lins[i] + thetaX[0];
	outputY[i] = thetaY[1]*lins[i] + thetaY[0];
#plt.scatter(lins, output,color = "y");
plt.scatter(X, Y);
plt.scatter(outputX, outputY, color = "r");
plt.show();
