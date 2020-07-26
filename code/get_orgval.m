function out = get_orgval(nzr_x, val)
% GET ORIGINAL VALUE

n = size(val, 1);
out = val.*repmat(nzr_x.sig+nzr_x.eps, n, 1) ...
    + repmat(nzr_x.mu, n, 1);
