function chain = fk_chain(chain,idx_to)
%
% Forward Kinematics
%
if nargin == 1
    idx_to = get_topmost_idx(chain); % start from the topmost joint
end

idx_fr = chain.joint(idx_to).parent;
if ~isempty(idx_fr)
    joint_fr = chain.joint(idx_fr);
    joint_to = chain.joint(idx_to);
    % update p
    chain.joint(idx_to).p = joint_fr.R*joint_to.p_offset + joint_fr.p; 
    % update R
    chain.joint(idx_to).R = joint_fr.R*joint_to.R_offset*rpy2r(joint_to.a*joint_to.q); 
end

% Recursive
for child = chain.joint(idx_to).childs
    chain = fk_chain(chain,child);
end
