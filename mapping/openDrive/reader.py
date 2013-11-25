import numpy as np
import matplotlib.pyplot as plt

C = np.loadtxt(open('output10', "rb"), delimiter = ',');
#get the x and z coordinates

plt.plot(C[:,0], C[:,1]);
plt.show();
