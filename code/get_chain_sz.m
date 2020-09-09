function chain_sz = get_chain_sz(chain)
%
% Get chain size
%

info_xyz_min = inf*ones(1,3); 
info_xyz_max = -inf*ones(1,3);
n_joint = length(chain.joint);
for j_idx = 1:n_joint % for all joints
    p_jt = chain.joint(j_idx).p';
    info_xyz_min = min(info_xyz_min,p_jt);
    info_xyz_max = max(info_xyz_max,p_jt);
end

% If link exist
if isfield(chain,'link')
    link = chain.link;
    n_link = length(link);
    
    for i_idx = 1:n_link
        link_i = link(i_idx);
        fv_i = link_i.fv;
        joint_idx = link_i.joint_idx;
        if ~isempty(fv_i )
            p_link = chain.joint(joint_idx).p;
            R_link = chain.joint(joint_idx).R;
            V = fv_i.vertices;
            V = p_link' + V*R_link';
            min_fv = min(V);
            max_fv = max(V);
            info_xyz_min = min(info_xyz_min,min_fv);
            info_xyz_max = max(info_xyz_max,max_fv);
        end
    end
end

chain_sz.xyz_min = info_xyz_min;
chain_sz.xyz_max = info_xyz_max;
chain_sz.xyz_len = info_xyz_max-info_xyz_min;
