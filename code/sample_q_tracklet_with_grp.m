function [q_tracklet,chain_model1,chain_model2] = sample_q_tracklet_with_grp(...
    chain_model,joi_model,t_sec,HZ,joint_vel_deg_max)
%
% Sample q_tracklet with GRP
%


% Get two collision-free poses
joint2use = chain_model.rev_joint_names; % joints to use
joint_limits = get_joint_limits(chain_model,joint2use); % joint limit info
[q1,chain_model1] = get_collision_free_random_pose(chain_model,joi_model,joint2use);
[q2,chain_model2] = get_collision_free_random_pose(chain_model,joi_model,joint2use); 

% Trim q2 based on linear interpolation
too_fast_rate = max(abs(q2-q1)*180/pi/t_sec/joint_vel_deg_max);
q2 = q1 + (q2-q1)/too_fast_rate;
chain_model2 = update_chain_q(chain_model2,chain_model2.rev_joint_names,...
    q2,'IGNORE_LIMIT',0);
chain_model2 = fk_chain(chain_model2); % forward kinematics

% Sample a trajectory using a GRP distribution connecting q1 to q2
n_test = t_sec*HZ; % # of test times
n_joint = length(joint2use);
q_tracklet = zeros(n_test,n_joint);
for j_idx = 1:n_joint % for each dim
    
    joint_range = [joint_limits.min(j_idx),joint_limits.max(j_idx)]; % joint limit
    
    % Define GRP
    q_fr = q1(j_idx); q_to = q2(j_idx); % start and final q
    t_anchor = [0,2]';
    x_anchor = [q_fr,q_to]';
    l_anchor = [1,1]';
    t_test = linspace(0,2,n_test)'; % test times
    hyp_mu = [1,1.0]; % [gain,len]
    gain_var = joint_range(2)-joint_range(1);
    hyp_var = [gain_var/2,1.0]; % [gain,len]
    eps_ru = 0.02; % epsilon run-up
    grp1d = init_grp1d(t_anchor,x_anchor,l_anchor,t_test,hyp_mu,hyp_var,'eps_ru',eps_ru);
    
    % Sample a trajectory
    randomness = randn(n_test,1);
    sampled_traj = sqrt(grp1d.K_max)*grp1d.chol_K*randomness + grp1d.mu_test;
    squashed_traj = get_squashed_traj(sampled_traj,joint_range(1),joint_range(2),10*pi/180);
    
    % Append
    q_tracklet(:,j_idx) = squashed_traj;
    
    % Plot each dim
    if 0
        ca;
        plot_grp1d(grp1d,sampled_traj,'fig_idx',1,'lw_sample',2,'axis_info','');
        plot(t_test,squashed_traj,'b-','linewidth',3);
        plot(t_test,joint_range(1)*ones(size(t_test)),'r--','linewidth',2);
        plot(t_test,joint_range(2)*ones(size(t_test)),'r--','linewidth',2);
        drawnow; pause;
    end
    
end