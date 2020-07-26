function chain = upfront_chain(chain,joi_mocap)

if joi_mocap.n == 0
    return
end

% Compute offset w.r.t the z-axis
rp = chain.joint(joi_mocap.idxs(idx_cell(joi_mocap.types,'rp'))).p;
lp = chain.joint(joi_mocap.idxs(idx_cell(joi_mocap.types,'lp'))).p;
theta_rad = atan2(rp(2)-lp(2),rp(1)-lp(1));
joint_rpy_rad = [0,0,270*pi/180-theta_rad]; 

% Apply offset to the root joint 
idx_offset  = get_topmost_idx(chain);
chain.joint(idx_offset).R = rpy2r(joint_rpy_rad)*chain.joint(idx_offset).R;

% FK 
chain = fk_chain(chain); 
