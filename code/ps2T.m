function T = ps2T(p1,p2,p3)
%
% Get a transformation matrix from three points in 3D
%
p = 1/3*(p1+p2+p3);
R = ps2R(p1,p2,p3);
T = pr2t(p,R);
