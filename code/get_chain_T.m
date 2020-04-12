function T = get_chain_T(chain,name)
%
% Get T of the kinematic chain
%

idx = idx_cell(chain.joint_names,name);
p = chain.joint(idx).p;
R = chain.joint(idx).R;
T = pr2t(p,R);
