# port of dllh2denu.m
from math import pi, cos, sin, sqrt
from numpy import array, power

def WGS84toENU(llh0, llh):
  ########CONSTANTS
  a = 6378137;
  b = 6356752.3142;
  e2 = 1 - (b/a)**2;
  ########Location of reference point in radians
  phi = llh0[0]*pi/180;
  lam = llh0[1]*pi/180; 
  h = llh0[2];
  ########Location of data points in radians
  dphi = llh[:,0]*pi/180 - phi;
  dlam = llh[:,1]*pi/180 - lam;
  dh = llh[:,2] - h;
  ########Some useful definitions
  tmp1 = sqrt(1-e2*power(sin(phi),2.0));
  cl = cos(lam);
  sl = sin(lam);
  cp = cos(phi);
  sp = sin(phi);
  ########Transformations
  de = (a/tmp1+h)*cp*dlam - (a*(1-e2)/(tmp1**3)+h)*sp*dphi*dlam +cp*dlam*dh;
  dn = (a*(1-e2)/tmp1**3 + h)*dphi + 1.5*cp*sp*a*e2*power(dphi,2) + sp**2*dh*dphi \
    + 0.5*sp*cp*(a/tmp1 +h)*power(dlam,2);
  du = dh - 0.5*(a-1.5*a*e2*cp**2+0.5*a*e2+h)*power(dphi,2) \
    - 0.5*cp**2*(a/tmp1 -h)*power(dlam,2);
  denu = array([de, dn, du]);
  return denu;


def WGS84toENU_slow(llh0, llh):
  ########CONSTANTS
  a = 6378137;
  b = 6356752.3142;
  e2 = 1 - (b/a)**2;
  ########Location of reference point in radians
  phi = llh0[0]*pi/180;
  lam = llh0[1]*pi/180; 
  h = llh0[2];
  ########Location of data points in radians
  dphi= llh[0]*pi/180 - phi;
  dlam= llh[1]*pi/180 - lam;
  dh = llh[2] - h;
  ########Some useful definitions
  tmp1 = sqrt(1-e2*sin(phi)**2);
  cl = cos(lam);
  sl = sin(lam);
  cp = cos(phi);
  sp = sin(phi);
  ########Transformations
  de = (a/tmp1+h)*cp*dlam - (a*(1-e2)/(tmp1**3)+h)*sp*dphi*dlam +cp*dlam*dh;
  dn = (a*(1-e2)/tmp1**3 + h)*dphi + 1.5*cp*sp*a*e2*dphi**2 + sp**2*dh*dphi \
    + 0.5*sp*cp*(a/tmp1 +h)*dlam**2;
  du = dh - 0.5*(a-1.5*a*e2*cp**2+0.5*a*e2+h)*dphi**2 \
    - 0.5*cp**2*(a/tmp1 -h)*dlam**2;
  denu = array([de, dn, du]);
  return denu;
  
def deg2rad(angleInDeg):
  return (pi / 180) * angleInDeg;

def rad2deg(angleInRad):
  return (180 / pi) * angleInRad; 
