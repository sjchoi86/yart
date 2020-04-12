function X = scew(x)
%
% Get a scew-symmetric matrix from a vector
%
X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];