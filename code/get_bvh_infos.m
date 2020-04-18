function mocap_infos = get_bvh_infos(varargin)

% Parse input arguments
p = inputParser;
addParameter(p,'mocap_folder','../mocap');
addParameter(p,'VERBOSE',false);
parse(p,varargin{:});
mocap_folder = p.Results.mocap_folder;
VERBOSE = p.Results.VERBOSE;

bvh_paths = dir([mocap_folder,'/**/*.bvh']); % get all bvh file paths
n_bvh = length(bvh_paths); % number of bvh file
mocap_infos = struct;
for i_idx = 1:n_bvh % for all files
    folder_i = bvh_paths(i_idx).folder;
    name_i = bvh_paths(i_idx).name;
    full_path = [folder_i,'/',name_i];
    temp = strsplit(name_i,{'-','_','.'}); % splite with '-'
    subject = temp{1};
    action = temp{2};
    if VERBOSE
        fprintf('[%03d/%d] subject:[%s] action:[%s] file name:[%s]. \n',...
            i_idx,n_bvh,subject,action, name_i);
    end
    mocap_infos(i_idx).subject = subject;
    mocap_infos(i_idx).action = action;
    mocap_infos(i_idx).full_path = full_path;
    %= {subject,action,full_path};
end
