function [enu] = xyz2enu(llh, XYZ)
  % convert ECEF coordinates to local east, north, up
    
  X = XYZ(:,1);
  Y = XYZ(:,2);
  Z = XYZ(:,3);
  
  refLat = llh(1)/180*pi;
  refLong = llh(2)/180*pi;
  refH = llh(3)/180*pi;
  
  % find reference location in ECEF coordinates
  XYZR = llh2xyz(llh);
  Xr = XYZR(1);
  Yr = XYZR(2);
  Zr = XYZR(3); 
  
  R = [-sin(refLong), cos(refLong), 0;
      -sin(refLat)*cos(refLong), -sin(refLat)*sin(refLong), cos(refLat);
      cos(refLat)*cos(refLong), cos(refLat)*sin(refLong), sin(refLat)]; 
  
  diff = bsxfun(@minus, XYZ, XYZR);
  enu = (R * diff')';
  
 
%   e = -sin(refLong).*(X-Xr) + cos(refLong).*(Y-Yr);
%   n = -sin(refLat)*cos(refLong).*(X-Xr) - sin(refLat)*sin(refLong).*(Y-Yr) + cos(refLat).*(Z-Zr);
%   u = cos(refLat)*cos(refLong).*(X-Xr) + cos(refLat)*sin(refLong).*(Y-Yr) + sin(refLat).*(Z-Zr);
%   enu = [e,n,u]