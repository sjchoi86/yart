function [SC,sc_ij_list,dists] = check_sc(chain_model,dist_th)

% dist_th: the bigger this value is, less collision checking

if nargin == 1
    dist_th = 0.0*max(chain_model.xyz_len)/20; % give some margin! 
end

SC = 0;
perm_list = randperm(size(chain_model.sc_checks,1));
sc_ij_list = [];
dists = [];
for sc_idx = perm_list
    sc_ij = chain_model.sc_checks(sc_idx,:);
    i = sc_ij(1);
    j = sc_ij(2);
    
    cap_i = chain_model.link(i).capsule;
    cap_j = chain_model.link(j).capsule;
    
    joint_idx_i = chain_model.link(i).joint_idx;
    joint_idx_j = chain_model.link(j).joint_idx;
    
    p_i = chain_model.joint(joint_idx_i).p;
    R_i = chain_model.joint(joint_idx_i).R;
    p_j = chain_model.joint(joint_idx_j).p;
    R_j = chain_model.joint(joint_idx_j).R;
    
    % Pre-filter out 
    dist = norm(p_i-p_j);
    if dist > cap_i.height+cap_i.radius+cap_j.height+cap_j.radius
        continue;
    end
    
    % Check collision
    dist = get_capsules_dist(cap_i,p_i,R_i,cap_j,p_j,R_j);
    if dist < -dist_th 
        SC = 1;
        sc_ij_list = [sc_ij_list; sc_ij];
        dists = [dists, dist]; 
        % return;
    end
end
