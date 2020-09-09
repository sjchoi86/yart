function chain_model = update_chain_q_fk(chain_model,jnames_ctrl,q)
%
% Update chain and do FK
%
chain_model = update_chain_q(chain_model,jnames_ctrl,q);
chain_model = fk_chain(chain_model);
