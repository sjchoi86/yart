%
% 1. Collect self-supervised glue data (q_tilde,x_tilde,q_bar,x_bar)
% 2. Collect domain-specific robot data (q_bar)
% 3. Collect domain-specific MoCap data (x_tilde)
% 4. Collect paired data from Rule-based Motion Retargeting (x_tilde -> q_bar)
% 
% 5. Check collected paired data using a simple nearest neighbor (NN) search
%
addpath_code
ccc
%% 1. Collect self-supervised glue data (q_tilde,x_tilde,q_bar,x_bar)
ccc

% Configuration
model_name = 'coman'; % atlas / coman /
n_sample = 50000; % # of samples
DO_PLOT = 0;

% Get the robot model
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);

% Sensitive analyasis of revolute joints
[~,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl);
joint_limits = get_joint_limits(chain_model,jnames_ctrl,'RESTRICT',2); % joint limits
reverse_sensitiveness = ''; % sample all joints 

% Now, we are ready to sample (q_tilde, x_tilde, q_bar, x_bar)
%
%  q_tilde ~ Q_{aggressive}    (DoF-dim)
%  x_tilde = FK(q_tilde)       (21-dim)
%  q_bar   = Proj(q_tilde)     (DoF-dim)
%  x_bar   = FK(q_bar)         (21-dim)
%
q_tildes = zeros(n_sample,joint_limits.n); x_tildes = zeros(n_sample,21);
q_bars = zeros(n_sample,joint_limits.n); x_bars = zeros(n_sample,21);
tk = init_tk(sprintf('Collect [%d] samples',n_sample));
rng('shuffle');
sample_cnt = 0;
while sample_cnt <= n_sample % for desired # of samples
    tk = print_tk(tk,sample_cnt,n_sample);
    
    % Sample
    q_tpose = zeros(1,11);
    % q_tpose = [0,0,0,0,70,0,-30,0,-70,0,-30]*D2R; % a-pose
    [q_tilde,x_tilde,q_bar,x_bar,chain_model_tilde,chain_model_bar,joi_tilde,joi_bar] = ...
        sample_qx_tilde_bar(...
        chain_model,joi_model,joint_limits,reverse_sensitiveness,jnames_ctrl,q_tpose);
    
    % Reject condition
    sims = [... 
        x_bar(1:3)*x_tilde(1:3)',x_bar(4:6)*x_tilde(4:6)',x_bar(7:9)*x_tilde(7:9)'...
        x_bar(10:12)*x_tilde(10:12)',x_bar(13:15)*x_tilde(13:15)',x_bar(16:18)*x_tilde(16:18)'...
        x_bar(19:21)*x_tilde(19:21)'];
    avg_sim = mean(sims); min_sim = min(sims);
    
    % Append
    if (min_sim > 0.7) && (avg_sim > 0.9)
        sample_cnt = sample_cnt + 1;
        q_tildes(sample_cnt,:) = q_tilde;
        x_tildes(sample_cnt,:) = x_tilde;
        q_bars(sample_cnt,:) = q_bar;
        x_bars(sample_cnt,:) = x_bar;
    end
    
    % Plot
    if DO_PLOT
        fprintf('avg_sim:[%.2f], min_sim:[%.2f].\n',avg_sim,min_sim);
        rs_deg = q_tilde(4)*R2D;
        ls_deg = q_tilde(8)*R2D;
        re_deg = q_tilde(7)*R2D;
        le_deg = q_tilde(11)*R2D;
        fprintf('Tilde  RE:[%.2f]deg LE:[%.2f]deg. RS:[%.2f]deg LS:[%.2f]deg.\n',...
            re_deg,le_deg,rs_deg,ls_deg);
        
        plot_qx_tilde_bar(chain_model_tilde,chain_model_bar,joi_tilde,joi_bar,...
            1,[0.0,0.4,0.9,0.6],[88,3],axis_info_ws+[0,0,0,3.0,0,0]); drawnow;
        [d_r2n,d_n2s,d_s2e,d_e2h] = get_link_lengths_chain(chain_model,joi_model);
        fig_pos2 = [0.0,0.2,0.3,0.3]; view_info = [88,16]; joi_sr = 0.03;
        plot_skeleton_from_features(x_tilde,'fig_idx',2,'subfig_idx',1,...
            'd_r2n',d_r2n,'d_n2s',d_n2s,'d_s2e',d_s2e,'d_e2h',d_e2h,...
            'fig_pos',fig_pos2,'view_info',view_info,...
            'color','r','joi_sr',joi_sr,'axis_info',axis_info_ws);
        plot_skeleton_from_features(x_bar,'fig_idx',2,'subfig_idx',2,...
            'd_r2n',d_r2n,'d_n2s',d_n2s,'d_s2e',d_s2e,'d_e2h',d_e2h,...
            'view_info',view_info,'color','k','joi_sr',joi_sr,'axis_info',axis_info_ws);
        drawnow;
        pause;
    end
end % for sample_cnt = 1:n_sample % for desired # of samples

% Save
dt = datestr(now,'mmmm-dd-yyyy HH-MM-SS');
mat_path = sprintf('data/Pairs of %s (%d) %s.mat',model_name,n_sample,dt);
save(mat_path,'q_tildes','x_tildes','q_bars','x_bars');
fprintf(2,'[%s] saved.\n',mat_path);

% Done.
fprintf('Done.\n');

%% 2. Collect domain-specific robot data (q_bar)
ccc

% Configuration
model_name = 'coman'; % atlas / coman /
n_sample = 50000; % # of samples
DO_PLOT = 0;

% Get the robot model
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);

% Sensitive analysis of revolute joints
[reverse_sensitiveness,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl);
joint_limits = get_joint_limits(chain_model,jnames_ctrl,'RESTRICT',1); 

% Sample feasble robot only
rng('shuffle');
q_bars = zeros(n_sample,joint_limits.n);
tk = init_tk(sprintf('Collect [%d] samples',n_sample));
sample_cnt = 0;
while sample_cnt <= n_sample % for desired # of samples
    tk = print_tk(tk,sample_cnt,n_sample);
    q = joint_limits.min + joint_limits.range.*rand(1,joint_limits.n);
    chain_model = update_chain_q(chain_model,jnames_ctrl,q,'IGNORE_LIMIT',0);
    chain_model = fk_chain(chain_model);
    SC = check_sc(chain_model);
    % Append the collision-free robot pose
    if SC == 0
        sample_cnt = sample_cnt + 1;
        q_bars(sample_cnt,:) = q;
    end
    % Plot
    if DO_PLOT
        xm = 0.15; ym = 0.15;
        plot_chain(chain_model,'fig_idx',1,'subfig_idx',1,'fig_pos',[0.0,0.6,0.3,0.4],...
            'view_info',[88,16],'axis_info',axis_info_ws,...
            'PLOT_LINK',0,'PLOT_ROTATE_AXIS',1,'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,...
            'PLOT_CAPSULE',0,...
            'mfc',[0.3,0.3,0.9],'mfa',0.1,'xm',xm,'ym',ym);
        drawnow; 
        pause;
    end
end

% Save
dt = datestr(now,'mmmm-dd-yyyy HH-MM-SS');
mat_path = sprintf('data/Domain of %s (%d) %s.mat',model_name,n_sample,dt);
save(mat_path,'q_bars');
fprintf(2,'[%s] saved.\n',mat_path);

% Done.
fprintf('Dchainone.\n');

%% 3. Collect domain-specific MoCap data (x_tilde)
ccc

% Configuration
n_sample = 50000; % number of samples to store
n_append_per_mocap = 500; % how many poses to append per mocap
DO_PLOT = 0;

% Get all bvh paths of CMU MoCap DB
mocap_infos = get_bvh_infos('mocap_folder','../../../cmu-mocap/');
rng('shuffle'); 
r_idxs = randperm(length(mocap_infos)); 
test_list = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];
m_list = setdiff(r_idxs,test_list); % exclude test set 


n_mocap = length(m_list);
fprintf('We will use [%d] mocaps out of [%d].\n',n_mocap,length(mocap_infos));

% Relaxed (all possible) skeletons
x_tildes = zeros(n_sample,21);

% Loop
sample_cnt = 0; % counter
rng('shuffle'); % rng shuffle
tk = init_tk(sprintf('Collect [%d] samples',n_sample));
while true
    tk = print_tk(tk,sample_cnt,n_sample);
    
    % Select 'n_append_per_mocap' random mocap poses
    m = mocap_infos(m_list(randi([1,n_mocap])));
    [chains_mocap,joi_mocap,skeleton,time] = get_chains_mocap_with_cache(...
        m,'RE',0,'cache_folder','../../cache');
    L = length(chains_mocap);
    r_idxs = randperm(L);
    sel_idxs = r_idxs(1:min(n_append_per_mocap,L));
    
    % Append
    for i_idx = sel_idxs
        chain_mocap = chains_mocap{i_idx}; % skeleton chain
        
        % x_skeleton = get_skeleton_of_chain_mocap(chain_mocap,joi_mocap);
        [p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck] = ...
            get_different_p_joi_mocap(chain_mocap,joi_mocap);
        % Add some perturbation?
        noise_std = 0.03;
        p_rs_perturb = p_rs + noise_std*randn(size(p_rs));
        p_re_perturb = p_re + noise_std*randn(size(p_re));
        p_rh_perturb = p_rh + noise_std*randn(size(p_rh));
        p_ls_perturb = p_ls + noise_std*randn(size(p_ls));
        p_le_perturb = p_le + noise_std*randn(size(p_le));
        p_lh_perturb = p_lh + noise_std*randn(size(p_lh));
        p_neck_perturb = p_neck + noise_std*randn(size(p_neck));
        % Form a skeleton feature
        x_skeleton = [...
            get_uv(p_neck_perturb-p_root); ... % root to neck
            get_uv(p_rs_perturb-p_neck_perturb); ... % neck to right shoulder
            get_uv(p_re_perturb-p_rs_perturb); ... % right shoulder to right elbow
            get_uv(p_rh_perturb-p_re_perturb); ... % right elbow to right hand
            get_uv(p_ls_perturb-p_neck_perturb); ... % neck to left shoulder
            get_uv(p_le_perturb-p_ls_perturb); ... % left shoulder to left elbow
            get_uv(p_lh_perturb-p_le_perturb); ... % left elbow to left hand
            ]';
        
        % Append
        sample_cnt = sample_cnt + 1;
        x_tildes(sample_cnt,:) = x_skeleton;
        
        if DO_PLOT
            plot_chain(chain_mocap);
            plot_spheres([p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck]',...
                'sr',0.03,'subfig_idx',2);
            [d_r2n,d_n2s,d_s2e,d_e2h] = get_link_lengths_chain(chain_mocap,joi_mocap);
            plot_skeleton_from_features(x_skeleton,'joi_sr',0.03,...
                'd_r2n',d_r2n,'d_n2s',d_n2s,'d_s2e',d_s2e,'d_e2h',d_e2h);
            drawnow; pause;
        end
    end
    
    % Termnate condition
    if sample_cnt > n_sample
        break;
    end
    
end

% Save
dt = datestr(now,'mmmm-dd-yyyy HH-MM-SS');
mat_path = sprintf('data/Domain of mocap (%d) %s.mat',n_sample,dt);
save(mat_path,'x_tildes');
fprintf(2,'[%s] saved.\n',mat_path);

% Done.
fprintf('Done.\n');

%% 4. Collect paired data from Rule-based Motion Retargeting (x_tilde -> q_bar)
ccc

% Configuration
model_name = 'coman';
DO_PLOT = 0;

% Select which mocap to use to collect paired data
mocap_infos = get_bvh_infos('mocap_folder','../../../cmu-mocap/');
rng('shuffle'); 
r_idx = randperm(length(mocap_infos)); 
mocap_idxs = r_idx(1:1000);
m_list = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];
mocap_idxs = setdiff(mocap_idxs,m_list); % exclude test set 

% Get the robot model
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);
[reverse_sensitiveness,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl);

% Loop
for m_idx = 1:length(mocap_idxs) % for all mocap data
    mocap_idx = mocap_idxs(m_idx );
    m = mocap_infos(mocap_idx);
    [~,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../../cmu-mocap/');
    [chains_mocap,joi_mocap,skeleton,time] = get_chains_mocap_with_cache(...
        m,'RE',0,'cache_folder','../../cache');
    L = length(chains_mocap);
    L_trim = 500;
    if L > L_trim % trim the maximum length of the trajectory 
        fprintf(' Trim the maximum length [%d]=>[%d].\n',L,L_trim);
        L = L_trim;
    end
    
    fprintf('[%d/%d] L:[%d] action_str:[%s].\n',...
        m_idx,length(mocap_idxs),L,action_str);
    
    mat_path = sprintf('data/Glue of CMU mocap %s-%s and %s.mat',...
        m.subject,m.action,model_name);
    SKIP_IF_MAT_EXISTS = 1;
    if exist(mat_path,'file') && SKIP_IF_MAT_EXISTS
        fprintf(2,'[%s] already exists. Skip.\n',mat_path);
        continue;
    end
    
    x_tildes=[]; x_bars = []; q_tildes = []; q_bars=[]; % data to store and save
    for tick = 1:L % for all tick
        chain_mocap = chains_mocap{tick};
        
        p_root = get_p_joi_type(chain_mocap,joi_mocap,'root');
        p_rh = get_p_joi_type(chain_mocap,joi_mocap,'rh');
        p_lh = get_p_joi_type(chain_mocap,joi_mocap,'lh');
        p_neck = get_p_joi_type(chain_mocap,joi_mocap,'neck');
        r2n = p_neck-p_root;
        r2n_deg = acos([0,0,1]*r2n/norm(r2n))*R2D;
        
        if r2n_deg > 30.0 % skip the mocap with too much crouch
            continue;
        end
        
        if tick == 1
            p_rh_prev = p_rh;
            p_lh_prev = p_lh;
        else
            rh_dist = norm(p_rh-p_rh_prev);
            lh_dist = norm(p_lh-p_lh_prev);
            hand_move = max([rh_dist,lh_dist]);
            if hand_move < 0.02 % skip the mocap with too small move (<2cm)
                continue;
            end
            p_rh_prev = p_rh; p_lh_prev = p_lh;
        end
        
        % Run motion retargeting
        r2n_at_ease_rate = 0.9; % root to neck at-ease rate (0.0~1.0)
        [q_tilde,mr_vec] = run_rule_based_simple_motion_retargeting(...
            chain_mocap,joi_mocap,chain_model,joi_model,jnames_ctrl,...
            'r2n_at_ease_rate',r2n_at_ease_rate,'q_init','');
        
        % Make the pose feasible
        chain_model = update_chain_q(chain_model,jnames_ctrl,q_tilde);
        chain_model = fk_chain(chain_model); % FK again
        [SC,sc_ij_list] = check_sc(chain_model); % check self-collision
        q_bar = q_tilde; % init q_bar
        q_tpose = zeros(size(q_tilde));
        sc_loop_cnt = 0;
        while SC % loop until collision-free
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
            chain_model = update_chain_q(chain_model,jnames_ctrl,q_bar,'IGNORE_LIMIT',0);
            chain_model = fk_chain(chain_model);
            [SC,sc_ij_list] = check_sc(chain_model); % check self-collision
        end % while SC % loop until collision-free
        
        % Final check with FK
        chain_model = update_chain_q(chain_model,jnames_ctrl,q_bar);
        chain_model = fk_chain(chain_model); % FK again
        SC = check_sc(chain_model);
        
        % Append paired data
        % Relaxed skeleton (from mocap) - Red
        x_tilde = get_skeleton_of_chain_mocap(chain_mocap,joi_mocap);
        % Feasible skeleton (from robot model) - Black
        x_bar = get_skeleton_of_chain_model(chain_model,joi_model);
        x_tildes = [x_tildes; x_tilde];
        x_bars = [x_bars; x_bar];
        q_tildes = [q_tildes; q_tilde'];
        q_bars = [q_bars; q_bar'];
        
        % Plot
        if DO_PLOT
            title_str = sprintf('[%s]-[%s] tick:[%d/%d] SC:[%d]',m.subject,m.action,tick,L,SC);
            plot_paired_data_from_rule_based_motion_retarget(...
                chain_model,joi_model,chain_mocap,joi_mocap,mr_vec,...
                x_tilde,x_bar,axis_info_ws,title_str);
        end
        
    end % for tick = 1:L % for all tick
    
    % Save
    save(mat_path,'x_tildes','x_bars','q_tildes','q_bars');
    fprintf(2,'[%s] saved. n:[%d].\n',mat_path,size(x_tildes,1));
    
end % for m_idx = 1:length(mocap_idxs) % for all mocap data

% Done.
fprintf('Done.\n');

%% 5. Check collected paired data using a simple nearest neighbor (NN) search
ccc

% Configuration
model_name = 'coman';

% Parse all paired data
USE_SSL_PAIRS = 1;
USE_MR_PAIRS  = 1;
x_tildes=[]; q_bars=[];

% 1. Self-supervised sampling
if USE_SSL_PAIRS
    dirs = dir(sprintf('data/Pairs of %s *',model_name));
    for i_idx = 1:length(dirs) % for i_idx = 1:length(dirs) % for all mat files
        mat_path = [dirs(i_idx).folder,'/',dirs(i_idx).name];
        l = load(mat_path);
        DO_AUGMENTATION = true;
        if DO_AUGMENTATION
            gammas = [0, 0.25, 0.5];
            for gamma = gammas
                x_tildes = [x_tildes; gamma*l.x_bars + (1-gamma)*l.x_tildes];
                q_bars = [q_bars; l.q_bars];
            end
        else
            x_tildes = [x_tildes; l.x_tildes];
            q_bars = [q_bars; l.q_bars];
        end
    end % for i_idx = 1:length(dirs) % for all mat files
end

% 2. Motion retargeting
if USE_MR_PAIRS
    dirs = dir(sprintf('data/Glue of CMU mocap*%s*',model_name));
    dirs = dirs(1:100); % use only 100 pairs
    for i_idx = 1:length(dirs) % for all mat files
        mat_path = [dirs(i_idx).folder,'/',dirs(i_idx).name];
        l = load(mat_path);
        DO_AUGMENTATION = true;
        if DO_AUGMENTATION
            gammas = [0, 0.25, 0.5];
            for gamma = gammas
                x_tildes = [x_tildes; gamma*l.x_bars + (1-gamma)*l.x_tildes];
                q_bars = [q_bars; l.q_bars];
            end
        else
            x_tildes = [x_tildes; l.x_tildes];
            q_bars = [q_bars; l.q_bars];
        end
    end % for i_idx = 1:length(dirs) % for all mat files
end

fprintf('We have total [%d] pairs.\n',size(x_tildes,1));

% Get the robot model
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);
[reverse_sensitiveness,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl);

% MoCap
mocap_infos = get_bvh_infos('mocap_folder','../../../cmu-mocap/');
m_list = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];
for m_idx = 1:length(m_list) % for different mocaps
    m = mocap_infos(m_list(m_idx));
    [chains_mocap,joi_mocap,skeleton,time] = get_chains_mocap_with_cache(...
        m,'RE',0,'cache_folder','../../cache');
    L = length(chains_mocap);
    fprintf('[%d/%d] [%s]-[%s] L:[%d].\n',m_idx,length(m_list),m.subject,m.action,L);
    
    for tick = 1:L % for each tick
        chain_mocap = chains_mocap{tick};
        x_skeleton = get_skeleton_of_chain_mocap(chain_mocap,joi_mocap);
        
        [p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck] = ...
            get_different_p_joi_mocap(chain_mocap,joi_mocap);
        if 0
            x_skeleton = [...
                get_uv(p_neck-p_root); ... % root to neck
                get_uv(p_rs-p_neck); ... % neck to right shoulder
                get_uv(p_re-p_rs); ... % right shoulder to right elbow
                get_uv(p_rh-p_re); ... % right elbow to right hand
                get_uv(p_ls-p_neck); ... % neck to left shoulder
                get_uv(p_le-p_ls); ... % left shoulder to left elbow
                get_uv(p_lh-p_le); ... % left elbow to left hand
                ]';
        end
        % Motion retarget with S3LE
        if 0
            [~,query_idx] = min(sum(abs(x_tildes-x_skeleton),2));
        else
            cos_sim_meas = ...
                0.1*x_tildes(:,1:3)*x_skeleton(1:3)' + ... % root to neck
                0.1*x_tildes(:,4:6)*x_skeleton(4:6)' + ... % neck to right shoulder
                x_tildes(:,7:9)*x_skeleton(7:9)' + ... % right shoulder to right elbow
                x_tildes(:,10:12)*x_skeleton(10:12)' + ... % right elbow to right hand
                0.1*x_tildes(:,13:15)*x_skeleton(13:15)' + ... % neck to left shoulder
                x_tildes(:,16:18)*x_skeleton(16:18)' + ... % left shoulder to left elbow
                x_tildes(:,19:21)*x_skeleton(19:21)'; % left elbow to left hand
            USE_WEIGHTED_SUM = 0;
            if USE_WEIGHTED_SUM
                [~,sort_idx] = sort(cos_sim_meas,'descend');
                query_idx = sort_idx(1:3);
            else
                [~,query_idx] = max(cos_sim_meas);
            end
        end
        q_star = mean(q_bars(query_idx,:),1);
        
        % Update model
        chain_model = update_chain_q(chain_model,jnames_ctrl,q_star,'IGNORE_LIMIT',0);
        chain_model = fk_chain(chain_model);
        
        % Plot
        plot_chain(chain_mocap,'subfig_idx',1,'PLOT_JOINT_AXIS',0,'axis_info',axis_info_ws);
        plot_chain(chain_model,'subfig_idx',2,'PLOT_JOINT_AXIS',0,'PLOT_CAPSULE',0,...
            'PLOT_ROTATE_AXIS',0);
        plot_spheres([p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck]','sr',0.02);
        drawnow;
        
    end % for tick = 1:L % for each tick
    
end % for m_idx = 1:length(m_list) % for different mocaps

fprintf('Done.\n');

%% 6. Prepare test data targets
ccc

% Configuration
model_name = 'coman';
DO_PLOT = 0;

% Select which mocap to use to collect paired data
mocap_infos = get_bvh_infos('mocap_folder','../../../cmu-mocap/');
mocap_idxs = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];

% Get the robot model
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);
[reverse_sensitiveness,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl);

% Loop
for m_idx = 1:length(mocap_idxs) % for all mocap data
    mocap_idx = mocap_idxs(m_idx );
    m = mocap_infos(mocap_idx);
    [~,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../../cmu-mocap/');
    [chains_mocap,joi_mocap,skeleton,time] = get_chains_mocap_with_cache(...
        m,'RE',0,'cache_folder','../../cache');
    L = length(chains_mocap);
    L_trim = 500;
    if L > L_trim % trim the maximum length of the trajectory 
        fprintf(' Trim the maximum length [%d]=>[%d].\n',L,L_trim);
        L = L_trim;
    end
    
    fprintf('[%d/%d] L:[%d] action_str:[%s].\n',...
        m_idx,length(mocap_idxs),L,action_str);
    
    mat_path = sprintf('data/Test Data of CMU mocap %s-%s and %s.mat',...
        m.subject,m.action,model_name);
    SKIP_IF_MAT_EXISTS = 1;
    if exist(mat_path,'file') && SKIP_IF_MAT_EXISTS
        fprintf(2,'[%s] already exists. Skip.\n',mat_path);
        continue;
    end
    
    x_tildes=[]; x_bars = []; q_tildes = []; q_bars=[]; % data to store and save
    for tick = 1:L % for all tick
        chain_mocap = chains_mocap{tick};
        
        p_root = get_p_joi_type(chain_mocap,joi_mocap,'root');
        p_rh = get_p_joi_type(chain_mocap,joi_mocap,'rh');
        p_lh = get_p_joi_type(chain_mocap,joi_mocap,'lh');
        p_neck = get_p_joi_type(chain_mocap,joi_mocap,'neck');
        r2n = p_neck-p_root;
        r2n_deg = acos([0,0,1]*r2n/norm(r2n))*R2D;
        
        if r2n_deg > 30.0 % skip the mocap with too much crouch
            continue;
        end
        
        if tick == 1
            p_rh_prev = p_rh;
            p_lh_prev = p_lh;
        else
            rh_dist = norm(p_rh-p_rh_prev);
            lh_dist = norm(p_lh-p_lh_prev);
            hand_move = max([rh_dist,lh_dist]);
            if hand_move < 0.02 % skip the mocap with too small move (<2cm)
                continue;
            end
            p_rh_prev = p_rh; p_lh_prev = p_lh;
        end
        
        % Run motion retargeting
        r2n_at_ease_rate = 0.9; % root to neck at-ease rate (0.0~1.0)
        [q_tilde,mr_vec] = run_rule_based_simple_motion_retargeting(...
            chain_mocap,joi_mocap,chain_model,joi_model,jnames_ctrl,...
            'r2n_at_ease_rate',r2n_at_ease_rate,'q_init','');
        
        % Make the pose feasible
        chain_model = update_chain_q(chain_model,jnames_ctrl,q_tilde);
        chain_model = fk_chain(chain_model); % FK again
        [SC,sc_ij_list] = check_sc(chain_model); % check self-collision
        q_bar = q_tilde; % init q_bar
        q_tpose = zeros(size(q_tilde));
        sc_loop_cnt = 0;
        while SC % loop until collision-free
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
            chain_model = update_chain_q(chain_model,jnames_ctrl,q_bar,'IGNORE_LIMIT',0);
            chain_model = fk_chain(chain_model);
            [SC,sc_ij_list] = check_sc(chain_model); % check self-collision
        end % while SC % loop until collision-free
        
        % Final check with FK
        chain_model = update_chain_q(chain_model,jnames_ctrl,q_bar);
        chain_model = fk_chain(chain_model); % FK again
        SC = check_sc(chain_model);
        
        % Append paired data
        % Relaxed skeleton (from mocap) - Red
        x_tilde = get_skeleton_of_chain_mocap(chain_mocap,joi_mocap);
        % Feasible skeleton (from robot model) - Black
        x_bar = get_skeleton_of_chain_model(chain_model,joi_model);
        x_tildes = [x_tildes; x_tilde];
        x_bars = [x_bars; x_bar];
        q_tildes = [q_tildes; q_tilde'];
        q_bars = [q_bars; q_bar'];
        
        % Plot
        if DO_PLOT
            title_str = sprintf('[%s]-[%s] tick:[%d/%d] SC:[%d]',m.subject,m.action,tick,L,SC);
            plot_paired_data_from_rule_based_motion_retarget(...
                chain_model,joi_model,chain_mocap,joi_mocap,mr_vec,...
                x_tilde,x_bar,axis_info_ws,title_str);
        end
        
    end % for tick = 1:L % for all tick
    
    % Save
    save(mat_path,'x_tildes','x_bars','q_tildes','q_bars');
    fprintf(2,'[%s] saved. n:[%d].\n',mat_path,size(x_tildes,1));
    
end % for m_idx = 1:length(mocap_idxs) % for all mocap data

% Done.
fprintf('Done.\n');

%%















