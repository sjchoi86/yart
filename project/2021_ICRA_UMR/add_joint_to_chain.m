function chain = add_joint_to_chain(chain,name,pos,parent_name)
%
% Add joint to chain
%

n = chain.n_joint;
n = n + 1; % increase counter
chain.n_joint = n;

% Update parent
if ~isempty(parent_name)
    parent_idx = idx_cell(chain.joint_names,parent_name);
    parent_childs = chain.joint(parent_idx).childs;
    parent_childs = [parent_childs, n]; % append parent child
    chain.joint(parent_idx).childs = parent_childs;
    parent_pos = chain.joint(parent_idx).p;
    % Update current
    chain.joint(n) = struct('name',name,'parent',[parent_idx],'childs',[],...
        'p',[0,0,0]','R',eye(3,3),'q',0,'a',[0,0,0]',...
        'p_offset',pos-parent_pos,'R_offset',rpy2r([0,0,0]));
else
    % Update current
    chain.joint(n) = struct('name',name,'parent',[],'childs',[],...
        'p',[0,0,0]','R',eye(3,3),'q',0,'a',[0,0,0]',...
        'p_offset',[0,0,0]','R_offset',rpy2r([0,0,0]));
end

% Append joint names
chain.joint_names{n} = name;

% FK chain
chain = fk_chain(chain);
