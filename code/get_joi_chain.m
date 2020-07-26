function joi = get_joi_chain(chain,varargin)
%
% Get Joints of interest (JOI) of a Chain
%

% Parse input options
p = inputParser;
addParameter(p,'retarget_option','upper-body'); % full-body / upper-body
parse(p,varargin{:});
retarget_option = p.Results.retarget_option;

switch lower(chain.name)
    case 'atlas'
        joi_types = {'rh','re','rs','lh','le','ls',... % hand, elbow, shoulder
            'root','rp','rk','ra','lp','lk','la','head'}; % root (pelvis), knee, ankle, head
        joi_idxs = [25,21,24,11,7,10,...
            3,30,33,29,16,19,14,20];
    case 'baxter'
        joi_types = {'rh','re','rs','lh','le','ls','head','root'}; % hand, elbow, shoulder, head
        joi_idxs = [24,20,17,42,38,35,10,2];
    case 'coman'
        joi_types = {'rh','re','rs','lh','le','ls',... % hand, elbow, shoulder
            'root','rp','rk','ra','lp','lk','la'}; % root (pelvis), knee, ankle, head
        joi_idxs = [48,38,35,44,42,39,...
            4,10,12,24,16,18,23];
    case 'darwin'
        joi_types = {'rh','re','rs','lh','le','ls',... % hand, elbow, shoulder
            'root','rp','rk','ra','lp','lk','la','head'}; % root (pelvis), knee, ankle, head
        joi_idxs = [17,14,12,10,7,5,...
            3,28,30,32,22,24,26,18];
    case 'isaac'
        joi_types = {'rh','re','rs','lh','le','ls',... % hand, elbow, shoulder
            'root','rp','rk','ra','lp','lk','la','head'}; % root (pelvis), knee, ankle, head
        joi_idxs = [27,24,22,36,33,31,...
            3,13,12,10,7,6,4,40];
    case 'kiwi'
        joi_types = {'rh','re','rs','lh','le','ls',... 
            'root','rp','rk','ra','lp','lk','la'}; 
        joi_idxs = [25,23,21,33,31,29,...
            3,11,13,15,5,7,9];
    case 'kiwi_upper_body'
        joi_types = {'rh','re','rs','lh','le','ls','root'}; 
        joi_idxs = [9,7,4,16,14,11,2];
    case 'nao'
        joi_types = {'rh','re','rs','lh','le','ls','head','root'}; % hand, elbow, shoulder, head
        joi_idxs = [31,29,27,25,23,21,3,19];
    case 'panda'
        joi_types = {'root','ee','rh','lh'}; 
        joi_idxs = [2,10,10,10];
    case 'sawyer'
        joi_types = {'root','ee','rh','lh'}; 
        joi_idxs = [2,18,18,18];
    case 'theo'
        joi_types = {'rh','re','rs','lh','le','ls',... 
            'root','head'}; 
        joi_idxs = [15,14,11,9,8,5,...
            3,31];
    case 'theo_with_cover'
        joi_types = {'rh','re','rs','lh','le','ls',... 
            'root','head'}; 
        joi_idxs = [15,14,11,9,8,5,... % [34,14,11,37,8,5,...
            17,31];
    case {'d1000','im'}
        joi_types = {'rh','re','rs','lh','le','ls',... 
            'root','head'}; 
        joi_idxs = [33, 30, 28, 25, 22, 20, 14, 36];
    case {'custom_humanoid','custom_01','custom_02','custom_03'}
        joi_types = {'rh','re','rs','lh','le','ls',... 
            'root','rp','rk','ra','lp','lk','la','head'}; 
        joi_idxs = [16,12,10,25,21,19,...
            3,31,33,34,37,39,40,27];
    case {'vibropt_arm','vibropt_arm_v2'}
        joi_types = {'ee'}; 
        joi_idxs = [7];
    case 'bork_noloop'
        joi_types = {'rh','re','rs','lh','le','ls',... 
            'root'}; 
        joi_idxs = [18, 16, 14, 12, 10, 8, 4];
    otherwise
        fprintf(2,'[get_joi_model] No JOI defined for [%s].\n',chain.name);
        joi_types = {};
        joi_idxs = {};
end

% For the upper-body only case 
if isequal(retarget_option,'upper-body')
    idx2rmv = [];
    types2rmv = {'rp','rk','ra','lp','lk','la'};
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
joi.types = joi_types;
joi.idxs = joi_idxs;
joi.n = length(joi_types);
