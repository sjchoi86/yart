function [chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(model_name)
%
% Get robot model information for motion retargeting
%

chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');
chain_model_sz = get_chain_sz(chain_model);
joi_model = get_joi_chain(chain_model);
ws = get_ws_with_cache(chain_model,joi_model,'ws_joi_types',{'rh','re','lh','le'},...
    'RE',0,'cache_folder','../cache','PLOT_EACH',0,'PLOT_FINAL',0);
jnames_ctrl = get_jnames_ctrl_from_joi_types(chain_model,joi_model,{'rh','lh'},'EXCLUDE_TARGET',1);