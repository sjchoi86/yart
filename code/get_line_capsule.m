function out = get_line_capsule (p,R,cap_opt)
% out = GETLINEFROMCAPSULE (h_cap)
% returns a line segment (two points) from a capsule handle

% calculate positions
% out.e1 = reshape(p,[3,1]) - 0.5*height*R*u;
% out.e2 = reshape(p,[3,1]) + 0.5*height*R*u;

T_cap = pr2t(cap_opt.p,cap_opt.R);
T_off = pr2t(p,R);
T = T_off*T_cap;
p = t2p(T);
R = t2r(T);
height = cap_opt.height;
u = [0 0 1]';

out.e1 = reshape(p,[3,1]) + 0.5*height*R*u;
out.e2 = reshape(p,[3,1]) - 0.5*height*R*u;
