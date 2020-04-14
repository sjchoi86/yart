function idx_route = get_idx_route(chain,joint_name_trgt)
%
% Get the list of indices from base to the target joint
%
idx_target = idx_cell(chain.joint_names,joint_name_trgt);
idx_route = [idx_target]; % start from target, which will be reversed 

idx_temp = idx_target;
while 1
    parent_idx = chain.joint(idx_temp).parent;
    idx_route = [idx_route; parent_idx]; % append parent
    idx_temp = parent_idx;
    if isempty(parent_idx)
        break;
    end
end
idx_route = flipud(idx_route); % flip to get base node to the end-effector
