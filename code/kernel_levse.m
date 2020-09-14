function [K,dK,ddK] = kernel_levse(X1, X2, L1, L2, hyp)
%
% Leveraged squared exponential kernel
%
d1 = size(X1, 2);
d2 = size(X2, 2);
if d1 ~= d2, fprintf(2, 'Data dimension missmatch! \n'); end

% Kernel hyperparameters
beta  = hyp(1);  % gain
gamma = hyp(2:end); % length parameters (the bigger, the smoother)

% Make each leverave vector a column vector
L1 = reshape(L1,[],1);
L2 = reshape(L2,[],1);

% Compute the leveraged kernel matrix
x_dists = pdist2(X1./gamma, X2./gamma, 'euclidean').^2;
l_dists = pdist2(L1, L2, 'cityblock');
K = beta * exp(-x_dists) .* cos(pi/2*l_dists);

% First derivative (assume xdim=1)
if nargin >= 1
    diff_X = bsxfun(@minus,X1./gamma,(X2./gamma)');
    dK = K .* (-2/gamma*diff_X);
end

% Second derivative (assume xdim=1)
if nargin >= 2
    ddK = dK .* (-2/gamma*diff_X) + K * (-2/(gamma^2));
end
