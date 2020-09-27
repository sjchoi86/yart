function [min_val,max_val,min_vel,max_vel,aa_vel] = get_path_stats(t_test,sampled_traj)
%
% Get some statistics of a trajectory
%


% Compute the velocity
n_anchor = 10;
[mu_test,dmu_test,t_data_vel,x_data_vel] = ...
    get_traj_vel(t_test,sampled_traj,t_test,n_anchor);

% Compute some statistics
min_val = min(mu_test);
max_val = max(mu_test);
min_vel = min(dmu_test);
max_vel = max(dmu_test);
aa_vel = mean(abs(dmu_test)); % absolute average velocity

