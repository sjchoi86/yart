function [ik_err,ik_idx] = compute_ik_err(chain,joint_name_target,p_trgt,R_trgt,IK_P,IK_R)
%
% Compute error for IK
%

% compute the error
joint_p = chain.joint(idx_cell(chain.joint_names,joint_name_target)).p; % current p
joint_R = chain.joint(idx_cell(chain.joint_names,joint_name_target)).R; % current R
p_err = p_trgt-joint_p;
R_err = joint_R \ R_trgt;
w_err = joint_R * r2w(R_err);
ik_err = [p_err; w_err];

% crop
ik_idx = [];
if IK_P
    ik_idx = [ik_idx, [1,2,3]];
end
if IK_R
    ik_idx = [ik_idx, [4,5,6]];
end
