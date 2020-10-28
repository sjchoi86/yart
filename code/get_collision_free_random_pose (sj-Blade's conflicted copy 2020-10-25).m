function [q,chain_model] = get_collision_free_random_pose(chain_model,joi_model,joint2use)
%
% Get collision-free random pose
%

while true
    % Get random q
    joint_limits = get_joint_limits(chain_model,joint2use);
    q = joint_limits.min + joint_limits.range.*rand(1,joint_limits.n);
    
    % Update model and check self-collision
    chain_model = update_chain_q(chain_model,chain_model.rev_joint_names,...
        q,'IGNORE_LIMIT',0);
    chain_model = fk_chain(chain_model); % forward kinematics
    SC = check_sc(chain_model);
    p = get_p_joi_type(chain_model,joi_model,'ee');
    if p(3) < 0, SC = 1; end
    
    % If collision-free, then use it
    if SC == 0
        break;
    end
end
