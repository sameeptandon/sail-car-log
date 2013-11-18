import pickle
import numpy as np
import matplotlib.pyplot as plt
#load the numpy file outputted by disp_2d.py
A = pickle.load(open('data.txt', 'rb'));

#get the x and z coordinates
C = A[:, [0,2]]

#plt.plot(C[:,0], C[:,1]);
#plt.show();
print C.shape
C = C[0:-1:100];
print C.shape
Next = C[1:];
Current = C[0:-1]
print Next.shape, Current.shape
Diff = Next - Current
Distance = np.sqrt(np.sum(Diff**2, axis =1))
Heading = np.arctan(Diff[:,1]/Diff[:,0]);
print Distance.shape


s = 0
start = C[0,:]
for i in range(3):
	start = C[i, :]
	s += Distance[i];
	st = "<geometry "
	st += 's ="{:.16e}" x="{:.16e}" y="{:.16e}" hdg="{:.16e}" length="{:.16e}"'.format(s, start[0], start[1], Heading[i], Distance[i]);
	st += ">\n<line/>"
	print st;
















