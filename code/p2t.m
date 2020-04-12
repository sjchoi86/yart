function T = p2t(p)
%
% Get T from p and R
%

R = eye(3,3);
T = [R,reshape(p,[3,1]);...
    0,0,0,1];
