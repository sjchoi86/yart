function K = kernel_ardmatern_bocd(X1, X2, bocd)
%
% ARD-kernel
%
nr_X1 = size(X1, 1);
nr_X2 = size(X2, 1);

% First, normalize inputs 
norm_factors = zeros(1, bocd.d);
for i = 1:bocd.d % for each dimension
    norm_factors(i) = (bocd.ranges(i).range(end) - bocd.ranges(i).range(1))*0.5;
end
norm_X1 = X1 ./ repmat(norm_factors+0.1, nr_X1, 1);
norm_X2 = X2 ./ repmat(norm_factors+0.1, nr_X2, 1);

% Compute Squared distances
rsq = sqdist(norm_X1, norm_X2);

% Compute Kernels
K = (1 + sqrt(5*rsq) + 5/3*rsq).*exp(-sqrt(5*rsq));

function d=sqdist(a_temp, b_temp)
% SQDIST - computes squared Euclidean distance matrix
%          computes a rectangular matrix of pairwise distances
% between points in A (given in rows) and points in B

% NB: very fast implementation taken from Roland Bunschoten
a = a_temp';
b = b_temp';

aa = sum(a.*a,1); bb = sum(b.*b,1); ab = a'*b; 
d = abs(repmat(aa',[1 size(bb,2)]) + repmat(bb,[size(aa,2) 1]) - 2*ab);
