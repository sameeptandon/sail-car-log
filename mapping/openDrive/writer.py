import pickle
import numpy as np
import matplotlib.pyplot as plt
#load the numpy file outputted by disp_2d.py
A = pickle.load(open('data.txt', 'rb'));

#get the x and z coordinates
C = A[:, [0,2]]

plt.plot(C[:,0], C[:,1]);
plt.show();
C = C[0:-1:100];
Next = C[1:];
Current = C[0:-1]
Diff = Next - Current
Distance = np.sqrt(np.sum(Diff**2, axis =1))
Heading = np.arctan(Diff[:,1]/Diff[:,0]);


s = 0
start = C[0,:]
for i in range(C.shape[0]-1):
	start = C[i, :]
	st = "<geometry "
	st += 's ="{:.16e}" x="{:.16e}" y="{:.16e}" hdg="{:.16e}" length="{:.16e}"'.format(s, start[0], start[1], Heading[i], Distance[i]);
	st += ">\n\t<line/>\n</geometry>"
	s += Distance[i];
	print st;

inertialB = 'minx:{:.16e} maxx:{:.16e} miny:{:.16e} maxy:{:.16e}'.format(np.min(C[:,0]), np.max(C[:,0]), np.min(C[:,1]),
		np.max(C[:,1]));

















