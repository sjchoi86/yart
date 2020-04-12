function T = r2t(R)
%
% Get a transformation matrix from a rotation matrix
%
T = pr2t([0,0,0]',R);