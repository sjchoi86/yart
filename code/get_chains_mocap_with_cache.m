function [chains_mocap,joi_mocap,skeleton,time] = get_chains_mocap_with_cache(m,varargin)
%
% Get motion capture chains
%

% Parse options
p = inputParser;
addParameter(p,'RE',false); % redo loading
addParameter(p,'cache_folder','../cache'); % folder to caache
parse(p,varargin{:});
RE = p.Results.RE;
cache_folder = p.Results.cache_folder;

% Cache file
cache_path = sprintf('%s/mocap/%s_%s.mat',cache_folder,m.subject,m.action);
[p,~,~] = fileparts(cache_path);
make_dir_if_not_exist(p);



if exist(cache_path,'file') && (RE==0)
    l = load(cache_path); % load
    chains_mocap = l.chains_mocap;
    joi_mocap = l.joi_mocap;
    skeleton = l.skeleton;
    time = l.time;
else
    bvh_path = m.full_path;
    [skeleton,time] = load_raw_bvh(bvh_path);
    chains_mocap = get_chain_from_skeleton(skeleton,time,...
        'USE_METER',1,'ROOT_AT_ORIGIN',1,'Z_UP',1,'HZ',30);
    joi_mocap = get_joi_mocap(m.subject); % joints of interest of MoCap
    L = length(chains_mocap);
    for tick = 1:L % for all tick, upfront
        chain_mocap = chains_mocap{tick};
        chain_mocap = upfront_chain(chain_mocap,joi_mocap);
        chains_mocap{tick} = chain_mocap;
    end
    
    % Save
    save(cache_path,'chains_mocap','joi_mocap','skeleton','time');
    fprintf(2,'[%s] saved.\n',cache_path);
end