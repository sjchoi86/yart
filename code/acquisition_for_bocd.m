function val = acquisition_for_bocd(input, bocd)
%
% Acquisition function for Bayesian Optimization with Expected Improvement Criteria (EIC)
%
n = bocd.n;
max_save = bocd.max_save;
if n > max_save
    [~,sort_idx] = sort(bocd.costs(1:bocd.n,:));
    X = bocd.inputs(sort_idx(1:max_save), :);
    Y = bocd.costs(sort_idx(1:max_save), :);
    n = max_save;
else
    X = bocd.inputs(1:bocd.n, :);
    Y = bocd.costs(1:bocd.n, :);
end

Y = (Y-mean(Y))/sqrt(var(Y));

xstar = input;
% KX = kernel_ardmatern(X, X, bo); % <= computed in 'add_bo'
% bo.KX = KX;
Kxstar = kernel_ardmatern_bocd(xstar, xstar, bocd);
KXxstar = kernel_ardmatern_bocd(X, xstar, bocd);

sig2w = 1e-4;
gp_var = Kxstar - KXxstar'/(bocd.KX + sig2w*eye(n, n))*KXxstar;
gp_var = diag(abs(gp_var));
mz_Y = Y - mean(Y);
gpmean = KXxstar'/(bocd.KX + sig2w*eye(n, n))*mz_Y + mean(Y);

fmin = min(Y);
gamma = (fmin - gpmean)./sqrt(gp_var);
Phi = cdf('Normal', gamma, 0, 1);
val = sqrt(gp_var) * gamma * Phi + pdf('Normal', gamma, 0, 1);
