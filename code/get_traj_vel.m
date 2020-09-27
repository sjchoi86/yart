function [mu_test,dmu_test,t_data_vel,x_data_vel] = get_traj_vel(t_list,x_list,t_test,n_anchor)
%
% Get the velocity of a trajectory
%

% Get anchor points
idx_anchor = round(linspace(1,size(x_list,1),n_anchor));
t_data_vel = t_list(idx_anchor,:);
x_data_vel = x_list(idx_anchor,:);

% GP-based velocity check
n_test = size(t_test,1);
hyp = [1,0.5]; % [gain,len]
[k_test,dk_test,~] = kernel_levse(t_test,t_data_vel,...
    ones(n_test,1),ones(n_anchor,1),hyp);
K_anchor = kernel_levse(t_data_vel,t_data_vel,...
    ones(n_anchor,1),ones(n_anchor,1),hyp);
meas_std = 1e-8; % expected noise
mu_test = k_test / (K_anchor+meas_std*eye(n_anchor,n_anchor)) * x_data_vel;
dmu_test = dk_test / (K_anchor+meas_std*eye(n_anchor,n_anchor)) * x_data_vel;

