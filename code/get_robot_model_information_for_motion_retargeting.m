function [chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(model_name,varargin)
%
% Get robot model information for motion retargeting
%

% Parse input options
p = inputParser;
addParameter(p,'RE',false); % redo loading
addParameter(p,'cache_folder','../cache'); % folder to caache
addParameter(p,'urdf_folder','../urdf'); % root folder contains URDF subfolders
parse(p,varargin{:});
RE = p.Results.RE;
cache_folder = p.Results.cache_folder;
urdf_folder = p.Results.urdf_folder;

% Get chain 
chain_model = get_chain_model_with_cache(...
    model_name,'RE',RE,'cache_folder',cache_folder,'urdf_folder',urdf_folder);

% Get more correct chain size and override 
chain_model_sz = get_chain_sz(chain_model);
chain_model.xyz_min = chain_model_sz.xyz_min;
chain_model.xyz_max = chain_model_sz.xyz_max;
chain_model.xyz_len = chain_model_sz.xyz_len;

% Get joints of interest
joi_model = get_joi_chain(chain_model);

% Get worksapce
ws = get_ws_with_cache(...
    chain_model,joi_model,'ws_joi_types',{'rh','re','lh','le'},...
    'RE',RE,'cache_folder',cache_folder,'PLOT_EACH',0,'PLOT_FINAL',0);

% Get joint names for control 
jnames_ctrl = get_jnames_ctrl_from_joi_types(...
    chain_model,joi_model,{'rh','lh'},'EXCLUDE_TARGET',1);
