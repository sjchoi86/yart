function out = rotate_vec(a,dir_vec,rotate_rate)
%
% Rotate a vector using a directional vector
%

mag = norm(a);
unit_v_in = a / mag;

dir_vec = reshape(dir_vec,[],1);
dir_vec = dir_vec / norm(dir_vec); % make it a unit vector

% rotated_v = (1-rotate_rate)*unit_v_in + rotate_rate*dir_vec;
rotated_v = slerp(unit_v_in,dir_vec,rotate_rate); % use slerp 

out = mag * rotated_v / norm(rotated_v);
