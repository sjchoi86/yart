function chain_mocap_llm = get_chain_mocap_llm(chain_mocap,joi_mocap,llm_stuct)
%
% Get Link Length Modified Chain
%

chain_mocap_llm = chain_mocap;
llm_names = fieldnames(llm_stuct);
for i_idx = 1:length(llm_names)
    llm_name = llm_names{i_idx};
    llm_rate = getfield(llm_stuct,llm_name);
    p_offset = chain_mocap_llm.joint(...
        joi_mocap.idxs(idx_cell(joi_mocap.types,llm_name))).p_offset;
    chain_mocap_llm.joint(joi_mocap.idxs(idx_cell(joi_mocap.types,llm_name))).p_offset = ...
        p_offset*llm_rate;
end
chain_mocap_llm = fk_chain(chain_mocap_llm);
