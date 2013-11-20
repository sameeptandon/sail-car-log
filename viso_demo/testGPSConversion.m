clear all
close all
clc
 
%% reference point
refLat = 39;
refLong = -132;
refH = 0;
ref = [refLat, refLong, refH]; 
 
%% Points of interest
lat = [39.1; 39.1;39.1; 39.3];
long = [-132;-131.1;-131.1; -131.3];
h = [0;0;1000;10];

target = [lat, long, h]; 
 
disp('lat long height')
for i = 1:length(lat)
    disp([num2str(lat(i)),' ', num2str(long(i)), ' ',num2str(h(i))])
end
% lat = [39.5*pi/180];
% long = [-132*pi/180];
% h = [0];
 
%% convering llh to enu
XYZ = llh2xyz(target);
disp('X Y Z')
 
for i = 1:size(XYZ,1)
    disp([num2str(XYZ(i,1)),' ', num2str(XYZ(i,2)), ' ',num2str(XYZ(i,3))])
end
 
enu = xyz2enu(ref, XYZ);
disp('e n u')
for i = 1:size(enu,1)
   disp([num2str(enu(i,1)),' ', num2str(enu(i,2)), ' ',num2str(enu(i,3))])
end
 
%% Converting enu to llh
XYZ = enu2xyz(ref, enu);
disp('X Y Z')
for i = 1:size(XYZ,1)
    disp([num2str(XYZ(i,1)),' ', num2str(XYZ(i,2)), ' ',num2str(XYZ(i,3))])
end
 
llh = xyz2llh(XYZ);
disp('lat long height')
for i = 1:size(llh,1)
    disp([num2str(llh(i,1)*180/pi),' ', num2str(llh(i,2)*180/pi), ' ',num2str(llh(i,3))])
end