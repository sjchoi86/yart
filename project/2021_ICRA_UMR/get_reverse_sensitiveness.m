function [reverse_sensitiveness,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl)
%
% Get reverse sensitiveness of joints
%

R2D = 180/pi;
D2R = pi/180;
n_joint = length(jnames_ctrl); % recompute the # of joints

% Sensitive analysis of revolute joints
[p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck] = ...
    get_different_p_joi_robot_model(chain_model,joi_model); % initial JOI positions
sensitiveness = zeros(1,n_joint);
for j_idx = 1:n_joint % for each moving joint
    sum_q_perturb = 0; sum_dev = 0; q_perturbs = [-5,-1,+1,+5]*D2R;
    for t_idx = 1:length(q_perturbs)
        % Perturb a single joint and check JOI deviations
        q_test = zeros(1,n_joint); q_perturb = q_perturbs(t_idx); q_test(j_idx) = q_perturb;
        chain_model = update_chain_q(chain_model,jnames_ctrl,q_test,'IGNORE_LIMIT',1);
        chain_model = fk_chain(chain_model);
        [p_root_n,p_rs_n,p_re_n,p_rh_n,p_ls_n,p_le_n,p_lh_n,p_neck_n] = ...
            get_different_p_joi_robot_model(chain_model,joi_model);
        dev = norm(p_rs_n-p_rs)+norm(p_re_n-p_re)+norm(p_rh_n-p_rh)+norm(p_ls_n-p_ls)+...
            norm(p_le_n-p_le)+norm(p_lh_n-p_lh)+norm(p_neck_n-p_neck);
        % Accumulate
        sum_q_perturb = sum_q_perturb + abs(q_perturb);
        sum_dev = sum_dev + dev;
    end
    avg_dev_per_q = sum_dev/sum_q_perturb; % avg JOI deviation per radian
    sensitiveness(j_idx) = avg_dev_per_q;
end
jnames_ctrl(sensitiveness==0) = []; % remove not affecting joint(s)
sensitiveness(sensitiveness==0) = []; % remove not affecting joint(s)
reverse_sensitiveness = ones(size(sensitiveness))./(sensitiveness+1); % between 0.0~1.0

