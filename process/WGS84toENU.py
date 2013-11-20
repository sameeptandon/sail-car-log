from numpy import pi, cos, sin, sqrt, arctan, arctan2
from numpy import array, power, dot

def WGS84toECEF(llh):
  lat = llh[:,0]*pi/180;
  lon = llh[:,1]*pi/180;
  h = llh[:,2]; 
  a = 6378137.0; # earth semimajor axis in meters
  f = 1/298.257223563; # reciprocal flattening
  e2 = 2.0*f -power(f,2.0); # eccentricity squared

  chi = sqrt(1-e2*power((sin(lat)),2));
  X = (a/chi +h)*cos(lat)*cos(lon);
  Y = (a/chi +h)*cos(lat)*sin(lon);
  Z = (a*(1-e2)/chi + h)*sin(lat);
  XYZ = array([X,Y,Z]);
  return XYZ

def ECEFtoENU(llh, XYZ):
  # convert ECEF coordinates to local east, north, up
  refLat = llh[0]/180*pi;
  refLong = llh[1]/180*pi;
  refH = llh[2]/180*pi;
  
  # find reference location in ECEF coordinates
  XYZR = WGS84toECEF(array([llh]));
  
  R = array([[-sin(refLong), cos(refLong), 0],
      [-sin(refLat)*cos(refLong), -sin(refLat)*sin(refLong), cos(refLat)],
      [cos(refLat)*cos(refLong), cos(refLat)*sin(refLong), sin(refLat)]]);  
  
  diff = XYZ - XYZR
  enu = dot(R, diff);
  return enu;


def ENUtoECEF(llh, enu):
  # Convert east, north, up coordinates (labeled e, n, u) to ECEF
  # coordinates. The reference point in WGS84 must be given. All distances are in metres
 
  e = enu[:,0];
  n = enu[:,1];
  u = enu[:,2]; 
  
  refLat = llh[0]/180*pi;
  refLong = llh[1]/180*pi; 
  refH = llh[2]/180*pi;
  
  XYZR = WGS84toECEF(array([llh]));
  R = array([[-sin(refLong), cos(refLong), 0],
      [-sin(refLat)*cos(refLong), -sin(refLat)*sin(refLong), cos(refLat)],
      [cos(refLat)*cos(refLong), cos(refLat)*sin(refLong), sin(refLat)]]);  
  XYZ = dot(R.transpose(), enu.transpose()) + XYZR; 
  return XYZ
  

def ECEFtoWGS84(XYZ):
  X = XYZ[:,0];
  Y = XYZ[:,1];
  Z = XYZ[:,2];

  a = 6378137.0; # earth semimajor axis in meters
  f = 1/298.257223563; # reciprocal flattening
  b = a*(1.0-f); # semi-minor axis
 
  e2 = 2.0*f-power(f,2.0); # first eccentricity squared
  ep2 = f*(2.0-f)/((1.0-f)**2); # second eccentricity squared
 
  r2 = power(X,2)+power(Y,2);
  r = sqrt(r2);
  E2 = power(a,2) - power(b,2);
  F = 54*power(b,2)*power(Z,2);
  G = r2 + (1.0-e2)*power(Z,2) - e2*E2;
  c = (e2*e2*F*r2)/(G*G*G);
  s = power((1.0 + c + sqrt(c*c + 2*c) ), 1.0/3.0);
  P = F/(3.0*power((s+1.0/s+1),2)*G*G);
  Q = sqrt(1+2*e2*e2*P);
  ro = -(e2*P*r)/(1.0+Q) + sqrt((a*a/2.0)*(1+1.0/Q) - ((1.0-e2)*P*power(Z,2))/(Q*(1.0+Q)) - P*r2/2.0);
  tmp = power((r - e2*ro),2);
  U = sqrt( tmp + power(Z,2) );
  V = sqrt( tmp + (1.0-e2)*power(Z,2) );
  zo = (power(b,2)*Z)/(a*V);
 
  h = U*( 1.0 - power(b,2)/(a*V));
  phi = arctan( (Z + ep2*zo)/r );
  lam = arctan2(Y,X);
  
  llh = array([180/pi*phi,180/pi*lam,h]);
  return llh

def WGS84toENU(llh0, llh):
  return ECEFtoENU(llh0, WGS84toECEF(llh))

def ENUtoWGS84(llh0, enu): 
  return ECEFtoWGS84(ENUtoECEF(llh0, enu).transpose())

def WGS84toENU_taylorexp(llh0, llh):
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
  # port of dllh2denu.m
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
