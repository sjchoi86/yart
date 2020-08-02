function sc_checks = get_sc_checks(chain_model)

sc_checks = [];
for i_idx = 1:chain_model.n_link
    for j_idx = i_idx+1:chain_model.n_link
        cap_i = chain_model.link(i_idx).capsule;
        cap_j = chain_model.link(j_idx).capsule;
        
        joint_idx_i = chain_model.link(i_idx).joint_idx;
        joint_idx_j = chain_model.link(j_idx).joint_idx;
        
        if isempty(joint_idx_i) || isempty(joint_idx_j) || ...
                isempty(cap_i) || isempty(cap_j) 
            continue;
        end
        p_i = chain_model.joint(joint_idx_i).p;
        R_i = chain_model.joint(joint_idx_i).R;
        p_j = chain_model.joint(joint_idx_j).p;
        R_j = chain_model.joint(joint_idx_j).R;
        capsule_sz_inc_rate = 1.1; % give more margin 
        dist = get_capsules_dist(cap_i,p_i,R_i,cap_j,p_j,R_j,capsule_sz_inc_rate);
        if dist > 0
            sc_checks = [sc_checks; i_idx,j_idx];
        end
    end
end
