function chain = update_chain_q(chain,names,qs)
%
% Update the position of the kinematic chain
%
for i_idx = 1:length(names)
    chain.joint(idx_cell(chain.joint_names,names{i_idx})).q = qs(i_idx);
end
