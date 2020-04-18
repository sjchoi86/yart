function J = compute_jacobian(chain,joint_names_to_control,joint_idxs_control,joint_name_target)
%
% Compute a Jacobian matrix
%
global TIME_PROFILING;

% Joints to Control for solving IK
iclk = clock;
if 0
    idx_ik = zeros(length(joint_names_to_control),1);
    for i_idx = 1:length(joint_names_to_control)
        joint_name_ik = joint_names_to_control{i_idx};
        idx_ik(i_idx) = idx_cell(chain.joint_names,joint_name_ik);
    end
else
    idx_ik = joint_idxs_control;
end
if TIME_PROFILING
    ems.joints_to_control = etime(clock,iclk)*1000;
end

% Current position of the joint
iclk = clock;
p_joint_current = chain.joint(idx_cell(chain.joint_names,joint_name_target)).p;
if TIME_PROFILING
    ems.position = etime(clock,iclk)*1000;
end

% Get the list of indices from root joint to the target joint
iclk = clock;
idx_route = get_idx_route(chain,joint_name_target);
if TIME_PROFILING
    ems.idx_route = etime(clock,iclk)*1000;
end

% Get the subset of indices
iclk = clock;
idx_use = intersect(idx_ik,idx_route); % this will actually be used for IK
n_use = length(idx_use);
if TIME_PROFILING
    ems.subset = etime(clock,iclk)*1000;
end

% Compute Jacobian
iclk = clock;
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
if TIME_PROFILING
    ems.jacobian = etime(clock,iclk)*1000;
end

if TIME_PROFILING
    fprintf([' [compute_jacobian] joints:[%.3f]ms position:[%.3f]ms idx_route:[%.3f]ms',...
        ' subset:[%.3f]ms jacobian[%.3f]ms\n'],...
        ems.joints_to_control,ems.position,ems.idx_route,ems.subset,ems.jacobian);
end