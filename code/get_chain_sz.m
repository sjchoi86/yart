function chain_sz = get_chain_sz(chain)

tick = 1;
info_xyz_min = inf*ones(1,3); 
info_xyz_max = -inf*ones(1,3);
n = length(chain.joint);
for i = 1:n
    p_jt = chain.joint(i).p(tick,:);
    info_xyz_min = min(info_xyz_min,p_jt);
    info_xyz_max = max(info_xyz_max,p_jt);
end

chain_sz.xyz_min = info_xyz_min;
chain_sz.xyz_max = info_xyz_max;
chain_sz.xyz_len = info_xyz_max-info_xyz_min;
