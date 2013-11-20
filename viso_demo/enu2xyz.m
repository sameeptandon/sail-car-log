function [XYZ] = enu2xyz(llh, enu)
  % Convert east, north, up coordinates (labeled e, n, u) to ECEF
  % coordinates. The reference point (phi, lambda, h) must be given. All distances are in metres
 
  e = enu(:,1);
  n = enu(:,2);
  u = enu(:,3); 
  
  refLat = llh(1)/180*pi;
  refLong = llh(2)/180*pi; 
  refH = llh(3)/180*pi;
  
  XYZR = llh2xyz(llh); % location of reference point
  
  R = [-sin(refLong), cos(refLong), 0;
      -sin(refLat)*cos(refLong), -sin(refLat)*sin(refLong), cos(refLat);
      cos(refLat)*cos(refLong), cos(refLat)*sin(refLong), sin(refLat)]; 
  
  XYZ = bsxfun(@plus, R' * enu', XYZR')'; 

%   X = -sin(refLong)*e - cos(refLong)*sin(refLat)*n + cos(refLong)*cos(refLat)*u + Xr;
%   Y = cos(refLong)*e - sin(refLong)*sin(refLat)*n + cos(refLat)*sin(refLong)*u + Yr;
%   Z = cos(refLat)*n + sin(refLat)*u + Zr;
%   XYZ = [X,Y,Z];