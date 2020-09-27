function chain = fk_chain(chain,idx_to)
%
% Forward Kinematics
%
if nargin == 1
    idx_to = get_topmost_idx(chain); % start from the topmost joint
end

if ~isfield(chain,'link')
    link_table = [];
elseif ~isempty(chain.link) && length(chain.link) > 1
    link_table = [0 chain.link(2:11).joint_idx]; % can be thought of a dictionary; {link_idx: associated joint_idx}
else
    link_table = [];
end

idx_fr = chain.joint(idx_to).parent;
if ~isempty(idx_fr)
    joint_fr = chain.joint(idx_fr);
    joint_to = chain.joint(idx_to);
    % update p
    chain.joint(idx_to).p = joint_fr.R*joint_to.p_offset + joint_fr.p;
    
    % update R
    if isfield(joint_to,'a') % this may cause some time delays.. (~10% load)
        q = joint_to.q;
        a = joint_to.a;
        chain.joint(idx_to).R = joint_fr.R*joint_to.R_offset*rodrigues(a,q);
    else
        chain.joint(idx_to).R = joint_fr.R*joint_to.R_offset;
    end
end

if ~isempty(link_table)
    link_idx_to = find(link_table == idx_to); % the index of the link attached to `joint_to`
    
    if ~isempty(link_idx_to)
        if ~isempty(chain.link(link_idx_to).bcube)
            % Center of Mass of each link represented in local joint coordinate system
            chain.joint(idx_to).com_local = chain.link(link_idx_to).bcube.c_offset;
            
            % Center of mass of each link represented in global coordinate system
            % Kajita (3.59)
            chain.link(link_idx_to).com = (chain.joint(idx_to).p + ...
                chain.joint(idx_to).R * chain.joint(idx_to).com_local);
        end
    end
end

% Recursive
for child = chain.joint(idx_to).childs
    chain = fk_chain(chain,child);
end
