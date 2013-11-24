import numpy as np
import matplotlib.pyplot as plt
from sklearn import linear_model

C = np.loadtxt(open('output10', "rb"), delimiter = ',');

#C = C[170:230,:]

X = C[:,0]
Y = C[:,1]
z = np.polyfit(X,Y, 30);
p = np.poly1d(z)
xp = np.linspace(-20, 20000, 10000)
plt.plot(X, Y, '.', xp, p(xp))

#regr = linear_model.LinearRegression()
#regr.fit(X, Y)
#Y_pred = regr.predict(X)
#plt.scatter(X, Y,  color='black')
#plt.plot(X, Y_pred, color='blue',
#linewidth=3)
#plt.scatter(X,Y);
plt.show();