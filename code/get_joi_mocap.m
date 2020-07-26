function joi_mocap = get_joi_mocap(mocap_subject,varargin)
%
% Define Joints of interest (JOI)
%

% Parse input options
p = inputParser;
addParameter(p,'retarget_option','upper-body'); % full-body / upper-body
parse(p,varargin{:});
retarget_option = p.Results.retarget_option;

CMU_MOCAP_BVH_SUBJECTS = cell(1,200);
for i_idx = 1:200
    CMU_MOCAP_BVH_SUBJECTS{i_idx} = sprintf('%02d',i_idx);
end
switch mocap_subject
    case 'Ralph'
        joi_idxs = [1, 41, 32, 30, 19, 10, 8, 56, 57, 58, 51, 52, 53, 5];
        joi_types = {'root','rh','re','rs','lh','le','ls','rp','rk','ra','lp','lk','la','head'};
    case {'jerry','james','kyna','robert','kevin'}
        joi_idxs = [1, 13, 12, 11, 9, 8, 7, 18, 19, 20, 14, 15, 16, 5];
        joi_types = {'root','rh','re','rs','lh','le','ls','rp','rk','ra','lp','lk','la','head'};
    case CMU_MOCAP_BVH_SUBJECTS % CMU mocaps in bvh formats (just numbers..?)
        joi_idxs = [1, 33, 32, 31, 24, 23, 22, ...
            9, 10, 11, 3, 4, 5, 19, 15, 16];
        joi_types = {'root','rh','re','rs','lh','le','ls',...
            'rp','rk','ra','lp','lk','la','head','spine1','spine2'};
    otherwise
        joi_idxs = [];
        joi_types = {};
        fprintf(2,'[get_joi_node] Unknown mocap name:[%s].\n',mocap_subject);
end

% For the upper-body only case
if isequal(retarget_option,'upper-body')
    idx2rmv = [];
    types2rmv = {'rk','ra','lk','la'};
    for i_idx = 1:length(types2rmv)
        temp_idx = idx_cell(joi_types,types2rmv{i_idx});
        if ~isempty(temp_idx)
            idx2rmv = [idx2rmv,temp_idx];
        end
    end
    joi_types(idx2rmv) = []; % remove elements from the cell
    joi_idxs(idx2rmv) = [];
end

% Output
joi_mocap.types = joi_types;
joi_mocap.idxs = joi_idxs;
joi_mocap.n = length(joi_types);
