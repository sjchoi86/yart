function grp1d = init_grp1d(t_anchor,x_anchor,l_anchor,t_test,hyp_mu,hyp_var,varargin)
%
% Initialize GRP 1D
%

% Parse options
p = inputParser;
addParameter(p,'eps_ru',''); % epsilon run-up
addParameter(p,'meas_noise_std',1e-8); % expected noise std
parse(p,varargin{:});
eps_ru = p.Results.eps_ru;
meas_noise_std = p.Results.meas_noise_std;

% # of test inputs
n_test = size(t_test,1);

% Add epsilon run-up to the anchor data
if ~isempty(eps_ru)
    [t_anchor,x_anchor,l_anchor] = add_eps_ru(t_anchor,x_anchor,l_anchor,eps_ru);
end
n_anchor = size(t_anchor,1);

% Compute GP mu
k_test_mu = kernel_levse(t_test,t_anchor,ones(n_test,1),ones(n_anchor,1),hyp_mu);
K_anchor_mu = kernel_levse(t_anchor,t_anchor,ones(n_anchor,1),ones(n_anchor,1),hyp_mu);
mu_test = k_test_mu / (K_anchor_mu+meas_noise_std*eye(n_anchor,n_anchor)) *...
    (x_anchor-mean(x_anchor)) + mean(x_anchor);

% Compute GP var
k_test_var = kernel_levse(t_test,t_anchor,ones(n_test,1),l_anchor,hyp_var);
K_anchor_var = kernel_levse(t_anchor,t_anchor,l_anchor,l_anchor,hyp_var);
eps_var = 1e-8;
var_test = diag(abs(hyp_var(1) - ...
    k_test_var / (K_anchor_var+eps_var*eye(n_anchor,n_anchor)) * k_test_var'));
std_test = sqrt(var_test);
K_test = kernel_levse(t_test,t_test,ones(n_test,1),ones(n_test,1),hyp_var) - ...
    k_test_var / (K_anchor_var+eps_var*eye(n_anchor,n_anchor)) * k_test_var';
K_test = 0.5*K_test + 0.5*K_test';
K_max = max(K_test(:));
chol_K = chol(K_test/sqrt(K_max) + 1e-6*eye(n_test,n_test),'lower');

% Save grp structure
grp1d.n_anchor = n_anchor;
grp1d.t_anchor = t_anchor;
grp1d.x_anchor = x_anchor;
grp1d.l_anchor = l_anchor;
grp1d.t_test = t_test;

grp1d.n_test = n_test;
grp1d.mu_test = mu_test;
grp1d.std_test = std_test;
grp1d.K_test = K_test;
grp1d.K_max = K_max;
grp1d.chol_K = chol_K;

