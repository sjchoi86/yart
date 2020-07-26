function jnames_ctrl = get_jnames_ctrl_from_joi_types(...
    chain_model,joi_model,joi_types,varargin)
%
% Get joint names to control from Joints of Interest
%
% Parse options
p = inputParser;
addParameter(p,'EXCLUDE_TARGET',false); % redo loading
parse(p,varargin{:});
EXCLUDE_TARGET = p.Results.EXCLUDE_TARGET;

idx_route = [];
for j_idx = 1:length(joi_types)
    joi_type = joi_types{j_idx};
    idx_route = [idx_route ; ...
        get_idx_route(...
        chain_model,get_joint_name_from_joi_type(chain_model,joi_model,joi_type),...
        'EXCLUDE_TARGET',EXCLUDE_TARGET)
        ];
end
idx_route = unique(idx_route);

jnames_ctrl = cell(1,length(idx_route)); n_ctrl = 0;
for j_idx = 1:length(idx_route)
    temp = chain_model.joint(idx_route(j_idx));
    if isequal(temp.type,'revolute') % only for revolute joints
        n_ctrl = n_ctrl + 1;
        jnames_ctrl{n_ctrl} = chain_model.joint(idx_route(j_idx)).name;
    end
end
jnames_ctrl = jnames_ctrl(1:n_ctrl);

