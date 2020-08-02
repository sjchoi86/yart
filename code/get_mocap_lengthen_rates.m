function [r2n_rate,n2s_rate,s2e_rate,e2h_rate] = ...
    get_mocap_lengthen_rates(chain_model,joi_model,chain_mocap,joi_mocap)
%
% Get lengthen rates of MoCap skeleton to match the robot morphology
%

% Get JOI Positions of the Robot (pr: position of robot)
[pr_root,pr_rs,pr_re,pr_rh,pr_ls,pr_le,pr_lh,pr_neck] = ...
    get_different_p_joi_robot_model(chain_model,joi_model);

% Get JOI Positions of MoCap (pm: position of mocap)
[pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck] = ...
    get_different_p_joi_mocap(chain_mocap,joi_mocap);

% Get Lengthen Rates
r2n_rate = norm(pr_neck-pr_root)/norm(pm_neck-pm_root);
n2s_rate = norm(pr_rs-pr_ls)/norm(pm_rs-pm_ls);
s2e_rate = norm(pr_re-pr_rs)/norm(pm_re-pm_rs);
e2h_rate = norm(pr_rh-pr_re)/norm(pm_rh-pm_re);
