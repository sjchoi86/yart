function q_traj_cf = get_q_traj_cf(chain_model,jnames_ctrl,q_traj,varargin)
%
% Get Collision-Free Joint Trajectories
%


% Parse options
p = inputParser;
addParameter(p,'TEMPORAL_CONSISTENCY',0);
parse(p,varargin{:});
TEMPORAL_CONSISTENCY = p.Results.TEMPORAL_CONSISTENCY;


q_traj_cf = q_traj;
L = size(q_traj,1);

for tick = 1:L
    
    q = q_traj(tick,:)';
    chain_model = update_chain_q_fk(chain_model,jnames_ctrl,q);
    SC = check_sc(chain_model);
    
    if (tick == 1) && SC % SC @tick=1
        q_prev = zeros(size(q)); % zero-pose
        t_res = 10;
        q_safe = get_q_safe(q,q_prev,t_res,chain_model,jnames_ctrl);
        q = q_safe;
    end
    
    if TEMPORAL_CONSISTENCY
        if (tick >= 2) && SC
            t_res = 10;
            q_safe = get_q_safe(q,q_prev,t_res,chain_model,jnames_ctrl);
            q = q_safe;
        end
    else
        if (tick >= 2) && SC
            q_prev = zeros(size(q)); % zero-pose
            t_res = 10;
            q_safe = get_q_safe(q,q_prev,t_res,chain_model,jnames_ctrl);
            q = q_safe;
        end
    end
    
    % Append
    q_traj_cf(tick,:) = q;
    q_prev = q;
end

function q_safe = get_q_safe(q,q_prev,t_res,chain_model,jnames_ctrl)
%
% Get q_safe
%
q_safe = q_prev;
for t_idx = 1:t_res
    alpha = t_idx / t_res;
    q_check = (1-alpha)*q_prev + alpha*q;
    chain_model_check = update_chain_q_fk(chain_model,jnames_ctrl,q_check);
    SC_check = check_sc(chain_model_check);
    if SC_check == 0
        q_safe = q_check;
    end
end

