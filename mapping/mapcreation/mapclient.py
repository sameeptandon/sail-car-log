import matplotlib.pyplot as plt
from mapper import Map
import numpy as np

#useful for scatterplots of coordinate data
def draw(data, color):
	plt.scatter(data[:,0], data[:,1], color= color);

#data - original and new
data = np.array([[5,1], [4.5,2], [3.75, 2.5], [3,3], [2.3, 3.4], [2, 4], [1.7, 5], [1.8,6], [2.4, 7], [3,8]]);
newData = np.array([[2.5, 2.6], [2.5, 7.5], [4.0, 2.4]]);

m = Map(data);
predCoords = m.generateMap();
#draw the original data
draw(data, "blue");

m.addToMap(newData);
newPredCoords = m.generateMap();
draw(newData, "black");
draw(predCoords, "red");
draw(newPredCoords, "green");
plt.show();
