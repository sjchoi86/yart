function J = compute_jacobian(chain,joint_names_to_control,joint_name_target)
%
% Compute a Jacobian matrix 
%

% Joints to Control for solving IK
idx_ik = zeros(length(joint_names_to_control),1);
for i_idx = 1:length(joint_names_to_control)
    joint_name_ik = joint_names_to_control{i_idx};
    idx_ik(i_idx) = idx_cell(chain.joint_names,joint_name_ik);
end

% Current position of the joint 
p_joint_current = chain.joint(idx_cell(chain.joint_names,joint_name_target)).p;

% Get the list of indices from root joint to the target joint
idx_route = get_idx_route(chain,joint_name_target);

% Get the subset of indices 
idx_use = intersect(idx_ik,idx_route); % this will actually be used for IK
n_use = length(idx_use);

% Compute Jacobian
n_ctrl = length(joint_names_to_control);
J = zeros(6,n_ctrl);
for i_idx = 1:n_use % along the joint route
    joint_idx = idx_use(i_idx);
    parent = chain.joint(joint_idx).parent; % parent joint
    R_offset = chain.joint(joint_idx).R_offset; % current joint's rotational offset
    a = chain.joint(parent).R * R_offset * chain.joint(joint_idx).a; % axis in the global coordinates
    jname_r = chain.joint_names{joint_idx};
    idx_append = idx_cell(joint_names_to_control,jname_r); % which column to append
    J(:,idx_append) = [cross(a',p_joint_current-chain.joint(joint_idx).p)';a];
end

