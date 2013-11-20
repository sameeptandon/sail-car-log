import WGS84toENU as tc
from numpy import array

p1 = array([ 3.750682775700000e+01, -1.223404661990000e+02, 7.872197645300000e+01])
p2 = array([3.750676376100000e+01,-1.223394834720000e+02,  7.980882372500000e+01])
z = array([p1,p2])
"""" scratch work, scroll down
print tc.WGS84toENU_slow(p1,p2)
print [8.689131850930292e+01, -7.102369271144839e+00, 1.086252189612837e+00]
print tc.WGS84toENU_taylorexp(p1,z)

XYZ_p2 = tc.WGS84toECEF(z);
ENU = tc.ECEFtoENU(p1, XYZ_p2)
XYZ_p2_back = tc.ENUtoECEF(p1, ENU.transpose())

print XYZ_p2
print XYZ_p2_back

z_back = tc.ECEFtoWGS84(XYZ_p2_back.transpose())
print z
print  z_back.transpose()
"""
#### here's a simple API

print '-----------------------'
print z
ENU_z = tc.WGS84toENU(p1, z)
print ENU_z
z_back = tc.ENUtoWGS84(p1, ENU_z.transpose())
print z_back.transpose()

