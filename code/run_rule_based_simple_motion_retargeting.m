function [q,mr_vec] = run_rule_based_simple_motion_retargeting(...
    chain_mocap,joi_mocap,chain_model,joi_model,jnames_ctrl)
%
% Run a simple rule-based motion retargeting
%

% Get JOI Positions of MoCap (pm: position of mocap)
[pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck] = ...
    get_different_p_joi_mocap(chain_mocap,joi_mocap);

% Get lengthen rates of MoCap skeleton to match the robot morphology
[r2n_rate,n2s_rate,s2e_rate,e2h_rate] = ...
    get_mocap_lengthen_rates(chain_model,joi_model,chain_mocap,joi_mocap);

% Get Modified JOI Positions for Motion Retargeting (mr: motion retargeting)
mr_vec = [0.5,0.0,0.0,0.0,r2n_rate,n2s_rate,s2e_rate,e2h_rate]; % rule-based MR
[pm_neck_mr,pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr] = ...
    get_p_joi_motion_retarget(pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck,mr_vec);

% Run IK
ik = init_ik(chain_model,'joint_names_control',jnames_ctrl);
ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'rh'),...
    'p',pm_rh_mr,'IK_P',1);
ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'re'),...
    'p',pm_re_mr,'IK_P',1);
ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'rs'),...
    'p',pm_rs_mr,'IK_P',1);
ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'lh'),...
    'p',pm_lh_mr,'IK_P',1);
ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'le'),...
    'p',pm_le_mr,'IK_P',1);
ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'ls'),...
    'p',pm_ls_mr,'IK_P',1);
q = (ik.joint_limits_lower+ik.joint_limits_upper)/2; % median pose
chain_model = update_chain_q(chain_model,ik.joint_names_control,q);
chain_model = fk_chain(chain_model); % initialize chain
while (ik.tick < 100) % loop until 500 ticks
    [ik,chain_model,q] = onestep_ik(ik,chain_model,q);
    [FLAG,ik,best_err] = check_ik_oscilating(ik); % check limbo
    if FLAG
        break;
    end
end
