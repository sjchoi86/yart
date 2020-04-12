function w = r2w(R)
%
% Get angular velocity in degree vector from a rotation matrix
%

if isequal(R,eye(3,3))
    w = [0,0,0]';
elseif isdiag(R)
    w = (pi/2)*[R(1,1),R(2,2),R(3,3)]';
else
    l = [R(3,2)-R(2,3),R(1,3)-R(3,1),R(2,1)-R(1,2)]';
    theta = atan2(norm(l), R(1,1)+R(2,2)+R(3,3)-1);
    w = theta * l / norm(l);
end

% To Degree
w = w * 180 / pi;