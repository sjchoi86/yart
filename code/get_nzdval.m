function out = get_nzdval(nzr_x, val)
%
% Get normalized value 
%

n = size(val, 1);
out = (val - repmat(nzr_x.mu, n, 1)) ...
    ./ repmat(nzr_x.sig+nzr_x.eps, n, 1);
