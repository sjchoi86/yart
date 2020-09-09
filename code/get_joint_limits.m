function joint_limits = get_joint_limits(chain_model,jnames_ctrl)
%
% Get joint limits in radian 
%

n_joint = length(jnames_ctrl);
q_min = zeros(1,n_joint); q_max = zeros(1,n_joint);
for j_idx = 1:n_joint % get joint limits
    limit = chain_model.joint(idx_cell(chain_model.joint_names,jnames_ctrl{j_idx})).limit;
    q_min(j_idx) = limit(1); q_max(j_idx) = limit(2);
end
q_range = q_max - q_min;

% Out
joint_limits.n = n_joint;
joint_limits.min = q_min;
joint_limits.max = q_max;
joint_limits.range = q_range;
