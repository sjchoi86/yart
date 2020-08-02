function idx = get_joint_idx_from_joi_type(joi_chain,joi_type)
%
% Get the joint index from the Joints of Interest Type
%

idx = joi_chain.idxs(idx_cell(joi_chain.types,joi_type));
