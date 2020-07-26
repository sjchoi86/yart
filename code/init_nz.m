function nzr_x = init_nz(data, eps)
%
% Gaussian normalizer 
%
if nargin == 1
    eps = 1e-8;
end

% Init normalizer
nzr_x.mu   = mean(data);
nzr_x.sig  = std(data);
nzr_x.eps  = eps;
% nzr_x.org_data = data;

zerodix = find(nzr_x.sig < 0.1);
if isempty(zerodix) == 0    
    nzr_x.sig(zerodix) = 1;
end
