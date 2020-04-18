function ik = get_ik_from_ik_config(chain,ik_config)
%
% Get IK structure from IK configuration
%
D2R = pi/180;

% Initialize IK with interactive marker
idxs = [];
for i_idx = 1:length(ik_config)
    idxs = union(idxs,get_idx_route(chain,ik_config(i_idx).name)); % all joints to targets
end
idxs = intersect(idxs,chain.rev_joint_idxs); % only for revolute joints
for i_idx = 1:length(ik_config)
    % idxs = setdiff(idxs,idx_cell(chain.joint_names,ik_config(i_idx).name)); % exclude target ones
end
joint_names_control = cell(1,length(idxs));
for i_idx = 1:length(idxs)
    joint_names_control{i_idx} = chain.joint(idxs(i_idx)).name;
end
ik = init_ik(chain,'joint_names_control',joint_names_control,...
    'stepsize',1.0*D2R,'stepsize_min',1.0*D2R,'stepsize_max',5.0*D2R,...
    'stepsize_inc_rate',2.0,'stepsize_dec_rate',0.5);
for i_idx = 1:length(ik_config)
    joint_name = ik_config(i_idx).name;
    ik = add_ik(ik,'joint_name',joint_name,...
        'p',chain.joint(idx_cell(chain.joint_names,joint_name)).p,...
        'R',chain.joint(idx_cell(chain.joint_names,joint_name)).R,...
        'IK_P',ik_config(i_idx).IK_P,'IK_R',ik_config(i_idx).IK_R);
end
