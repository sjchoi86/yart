function chain = update_chain_q(chain,names,q,varargin)
%
% Update the position of the kinematic chain
% 
% q: joint angles in radian
%

% Parse options
p = inputParser;
addParameter(p,'IGNORE_LIMIT',0);
parse(p,varargin{:});
IGNORE_LIMIT = p.Results.IGNORE_LIMIT;

q = mod(q+pi,2*pi)-pi; % -pi to +pi

for i_idx = 1:length(names)
    joint_idx = idx_cell(chain.joint_names,names{i_idx});
    if isfield(chain.joint(joint_idx),'limit')
        limit = chain.joint(joint_idx).limit;
    else
        limit = [-inf,+inf];
    end
    if IGNORE_LIMIT
        chain.joint(joint_idx).q = q(i_idx); % ignore joint limits
    else
        chain.joint(joint_idx).q = min(max(q(i_idx),limit(1)),limit(2)); % trim
    end
    
    % Compute the position difference
    if isfield(chain.joint(joint_idx),'q_prev')
        chain.joint(joint_idx).q_diff = chain.joint(joint_idx).q - chain.joint(joint_idx).q_prev;
        chain.joint(joint_idx).q_prev = chain.joint(joint_idx).q;
    end
end
