function joint_limits = get_joint_limits(chain_model,jnames_ctrl,varargin)
%
% Get joint limits in radian
%

% Parse options
p = inputParser;
addParameter(p,'RESTRICT',0);
parse(p,varargin{:});
RESTRICT = p.Results.RESTRICT;


n_joint = length(jnames_ctrl);
q_min = zeros(1,n_joint); q_max = zeros(1,n_joint);
for j_idx = 1:n_joint % get joint limits
    jname = jnames_ctrl{j_idx};
    limit = chain_model.joint(idx_cell(chain_model.joint_names,jname)).limit;
    
    if RESTRICT >= 1
        if isequal(chain_model.name,'coman') % reduce the range of hip 3-DoF joints
            switch jname
                case 'WaistLat'
                    limit = limit*0.1;
                case 'WaistSag'
                    limit = limit*0.1;
                case 'WaistYaw'
                    limit = limit*0.1;
            end
        end
    end
    if RESTRICT >= 2
        if isequal(chain_model.name,'coman') % reduce the range of hip 3-DoF joints
            switch jname
                case 'RShYaw'
                    limit = limit*0.1;
                case 'LShYaw'
                    limit = limit*0.1;
                case 'RShSag'
                    limit = limit*0.2;
                case 'LShSag'
                    limit = limit*0.2;
            end
        end
    end
    
    q_min(j_idx) = limit(1); q_max(j_idx) = limit(2);
end
q_range = q_max - q_min;

% Out
joint_limits.n = n_joint;
joint_limits.min = q_min;
joint_limits.max = q_max;
joint_limits.range = q_range;

