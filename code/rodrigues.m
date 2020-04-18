function R = rodrigues(a,q)
%
% Compute the rotation matrix from an angular velocity vector
%  The reverse (R -> w) can be achieve by the r2w function where w = a*q
%

if abs(norm(a)) < 1e-6 
    R = eye(3,3);
    return;
end

if abs(norm(a)-1) > 1e-6
    % fprintf(2,'[rodrigues] a is not a unit vector (%.4e).\n',norm(a));
end

% Resolve some numerical issues
norm_a = norm(a);
a = a / norm_a;
q = q * norm_a;

% Get R
a_hat = scew(a);
R = eye(3,3) + a_hat*sin(q) + a_hat*a_hat*(1-cos(q));
