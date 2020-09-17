function chain = fv_chain(chain,idx_to)
%
% Forward Velocities
%
% We need 'q_diff' and 'dt' to be pre-determined. 
%
if nargin == 1
    idx_to = get_topmost_idx(chain); % start from the topmost joint
end

if ~isempty(chain.link) && length(chain.link) > 1
    link_table = [0 chain.link(2:11).joint_idx]; % can be thought of a dictionary; {link_idx: associated joint_idx}
else
    link_table = [];
end

idx_fr = chain.joint(idx_to).parent;
if ~isempty(idx_fr)
    joint_fr = chain.joint(idx_fr);
    joint_to = chain.joint(idx_to);
    chain.joint(idx_to).v = joint_fr.v + cross(joint_fr.w,joint_fr.R*joint_to.p_offset);          % Kajita (3.61)
    chain.joint(idx_to).w = joint_fr.w + (joint_to.R * joint_to.a * (joint_to.q_diff/chain.dt) ); % Katija (3.62)
    
    if ~isempty(link_table) && ~isempty(joint_fr.com_local)
        link_idx_to = find(link_table == idx_fr); % the index of the link attached to `joint_to`
        chain.link(link_idx_to).v = joint_fr.v + cross(joint_fr.w, joint_fr.R*joint_fr.com_local.'); % com velocity - Kajita (3.64)
        chain.link(link_idx_to).w = joint_fr.w;
    end
end

% Recursive
for child = chain.joint(idx_to).childs
    chain = fv_chain(chain,child);
end
