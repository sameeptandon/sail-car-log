from WGS84toENU import *
from numpy import array

p1 = ( 3.750682775700000e+01, -1.223404661990000e+02, 7.872197645300000e+01)
d = WGS84toENU(p1,p1)
p2 = (3.750676376100000e+01,-1.223394834720000e+02,  7.980882372500000e+01)
print WGS84toENU(p1,p2)
print [8.689131850930292e+01, -7.102369271144839e+00, 1.086252189612837e+00]



