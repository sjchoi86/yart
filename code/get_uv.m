function uv = get_uv(x)
%
% Get unit vector
%

EPS = 1e-8;
norm_x = norm(x);
if norm_x < EPS
    uv = zeros(size(x));
else
    uv = x / norm_x;
end
