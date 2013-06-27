function denu = dllh2denu(llh0,llh)
%%%%%%%%%%CONSTANTS
a = 6378137;
b = 6356752.3142;
e2 = 1 - (b/a)^2;
%%%%%%%%%%Location of reference point in radians
phi = llh0(1)*pi/180;
lam = llh0(2)*pi/180; 
h = llh0(3);
%%%%%%%%%%Location of data points in radians
dphi= llh(:,1)*pi/180 - phi;
dlam= llh(:,2)*pi/180 - lam;
dh = llh(:,3) - h;
%%%%%%%%%%Some useful definitions
tmp1 = sqrt(1-e2*sin(phi)^2);
cl = cos(lam);
sl = sin(lam);
cp = cos(phi);
sp = sin(phi);
%%%%%%%%%%Transformations
de = (a/tmp1+h)*cp*dlam - (a*(1-e2)/(tmp1^3)+h)*sp.*dphi.*dlam +cp.*dlam.*dh;
dn = (a*(1-e2)/tmp1^3 + h)*dphi + 1.5*cp*sp*a*e2*dphi.^2 + sp^2.*dh.*dphi + ...
    0.5*sp*cp*(a/tmp1 +h)*dlam.^2;
du = dh - 0.5*(a-1.5*a*e2*cp^2+0.5*a*e2+h)*dphi.^2 - ...
    0.5*cp^2*(a/tmp1 -h)*dlam.^2;
denu = [de, dn, du];
end