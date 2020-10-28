function [d_r2n,d_n2s,d_s2e,d_e2h] = get_link_lengths_chain(chain,joi_chain)
%
% Get some link length of the kinematic chain 
%

p_root = chain.joint(get_joint_idx_from_joi_type(joi_chain,'root')).p;
p_rs = chain.joint(get_joint_idx_from_joi_type(joi_chain,'rs')).p;
p_ls = chain.joint(get_joint_idx_from_joi_type(joi_chain,'ls')).p;
p_re = chain.joint(get_joint_idx_from_joi_type(joi_chain,'re')).p;
p_rh = chain.joint(get_joint_idx_from_joi_type(joi_chain,'rh')).p;
p_neck = 0.5*(p_rs+p_ls);

d_r2n = norm(p_root-p_neck);
d_n2s = norm(p_neck-p_rs);
d_s2e = norm(p_rs-p_re);
d_e2h = norm(p_re-p_rh);
