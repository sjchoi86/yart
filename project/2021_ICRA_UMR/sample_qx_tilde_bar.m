function [q_tilde,x_tilde,q_bar,x_bar,chain_model_tilde,chain_model_bar,joi_tilde,joi_bar] = ...
    sample_qx_tilde_bar(...
    chain_model,joi_model,joint_limits,reverse_sensitiveness,jnames_ctrl,q_tpose)
%
% Sample q_tilde, x_tilde, q_bar, x_bar
%

% Sampling configuration
jrr  = 1.0; % joint randomness rate (0.0~1.0)
jlvr = 0.2; % joint limit violate rate (0.0~1.0)
n_joint = length(jnames_ctrl); % recompute the # of joints

% 1. Randomly sample joint angles with violating joint limits (q_tilde)
q_min = joint_limits.min; 
q_max = joint_limits.max; 
q_range = joint_limits.range;
q_tilde = (q_min-0.5*jlvr*q_range) + (q_range+jlvr*q_range).*rand(1,n_joint);
if ~isempty(reverse_sensitiveness)
    tpose_mask = (reverse_sensitiveness < rand(size(reverse_sensitiveness)));
    q_tilde = jrr*q_tilde + (1-jrr)*q_tpose;
    q_tilde = tpose_mask.*q_tpose + (1-tpose_mask).*q_tilde; % mask highly sensitive joints
end

% Forward kinematics (without considering constraints)
chain_model = update_chain_q(chain_model,jnames_ctrl,q_tilde,'IGNORE_LIMIT',1);
chain_model_tilde = fk_chain(chain_model);
[p_root_tilde,p_rs_tilde,p_re_tilde,p_rh_tilde,p_ls_tilde,p_le_tilde,p_lh_tilde,...
    p_neck_tilde] = ...
    get_different_p_joi_robot_model(chain_model_tilde,joi_model);
joi_tilde = [p_root_tilde,p_rs_tilde,p_re_tilde,p_rh_tilde,p_ls_tilde,p_le_tilde,p_lh_tilde,...
    p_neck_tilde]';

% 2. Get the corresponding skeleton of the robot (x_tilde = FK(q_tilde))
%
% Skeleton parametrization (21-dimensional vector):
%  unit vectors of [h2n,n2rs,rs2re,re2rh,n2ls,ls2le,le2lh]
%
x_tilde = get_skeleton_of_chain_model(chain_model_tilde,joi_model);

% 3. Project q_tilde into the feasible joint space (q_bar = Proj(q_tilde))
%
% Feasible in terms of both joint limit constraints and self-collision free.
%
q_bar = min(max(q_tilde,q_min),q_max);
chain_model = update_chain_q(chain_model,jnames_ctrl,q_bar,'IGNORE_LIMIT',0);
chain_model_bar = fk_chain(chain_model);
[SC,sc_ij_list] = check_sc(chain_model_bar); % check self-collision
MAKE_Q_BAR_COLLISION_FREE = 1; sc_loop_cnt = 0;
while (SC && MAKE_Q_BAR_COLLISION_FREE) % until collision-free
    sc_loop_cnt = sc_loop_cnt + 1;
    sc_link_idxs = unique(sc_ij_list(:));
    jnames_to_move = [];
    for l_idx = 1:length(sc_link_idxs)
        sc_link_idx = sc_link_idxs(l_idx);
        joint_idx = chain_model.link(sc_link_idx).joint_idx;
        joint_name = chain_model.joint_names{joint_idx};
        idx_route = get_idx_route(chain_model,joint_name);
        jnames_to_move = [jnames_to_move,chain_model.joint_names(idx_route)];
    end
    jnames_to_move = unique(jnames_to_move); % joints to control to make collision-free
    [jnames_to_move,~,match_idx] = intersect(jnames_to_move,jnames_ctrl);
    % Weighted average with T-pose to make it collision-free
    mix_ratio = 0.95;
    q_bar(match_idx) = mix_ratio*q_bar(match_idx) + (1-mix_ratio)*q_tpose(match_idx);
    chain_model_bar = update_chain_q(chain_model_bar,jnames_ctrl,q_bar,'IGNORE_LIMIT',0);
    chain_model_bar = fk_chain(chain_model_bar);
    [SC,sc_ij_list] = check_sc(chain_model_bar); % check self-collision
end

% 4. Get x_bar = FK(q_bar)
[p_root_bar,p_rs_bar,p_re_bar,p_rh_bar,p_ls_bar,p_le_bar,p_lh_bar,p_neck_bar] = ...
    get_different_p_joi_robot_model(chain_model_bar,joi_model);
joi_bar = [p_root_bar,p_rs_bar,p_re_bar,p_rh_bar,p_ls_bar,p_le_bar,p_lh_bar,p_neck_bar]';
x_bar = get_skeleton_of_chain_model(chain_model_bar,joi_model);


% Re-compute q
q_tilde = get_q_chain(chain_model_tilde,jnames_ctrl)';
q_bar = get_q_chain(chain_model_bar,jnames_ctrl)';



