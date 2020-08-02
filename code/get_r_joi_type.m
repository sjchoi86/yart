function R = get_r_joi_type(chain,joi_chain,joi_type)
%
% Get the rotation of the chain from JOI type
%

R = chain.joint(get_joint_idx_from_joi_type(joi_chain,joi_type)).R;