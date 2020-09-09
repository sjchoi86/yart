function chain = fv_chain(chain,idx_to)
%
% Forward Velocities
%
if nargin == 1
    idx_to = get_topmost_idx(chain); % start from the topmost joint
end

idx_fr = chain.joint(idx_to).parent;
if ~isempty(idx_fr)
    joint_fr = chain.joint(idx_fr);
    joint_to = chain.joint(idx_to);
    chain.joint(idx_to).v = joint_fr.v + cross(joint_fr.w,joint_fr.R*joint_to.p_offset);
    chain.joint(idx_to).w = joint_fr.w + joint_fr.R*(joint_to.a * (joint_to.q_diff/chain.dt) );
end

% Recursive
for child = chain.joint(idx_to).childs
    chain = fv_chain(chain,child);
end