function name = get_joint_name_from_joi_type(chain,joi_chain,joi_type)
%
% Get the joint index from the Joints of Interest Type
%

idx = joi_chain.idxs(idx_cell(joi_chain.types,joi_type));
name = chain.joint(idx).name;