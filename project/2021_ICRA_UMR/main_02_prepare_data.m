addpath_code
ccc
%% 1. Collect self-supervised glue data (q_tilde,x_tilde,q_bar,x_bar)
ccc

% Configuration
model_name = 'coman'; % atlas / coman /
n_sample = 50000; % # of samples
DO_PLOT = 1;

% Get the robot model
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);

% Sensitive analysis of revolute joints
[reverse_sensitiveness,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl);
joint_limits = get_joint_limits(chain_model,jnames_ctrl); % joint limits


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
for sample_cnt = 1:n_sample % for desired # of samples
    tk = print_tk(tk,sample_cnt,n_sample);
    
    % Sample
    [q_tilde,x_tilde,q_bar,x_bar,chain_model_tilde,chain_model_bar,joi_tilde,joi_bar] = ...
        sample_qx_tilde_bar(...
        chain_model,joi_model,joint_limits,reverse_sensitiveness,jnames_ctrl);
    
    % Append
    q_tildes(sample_cnt,:) = q_tilde; x_tildes(sample_cnt,:) = x_tilde;
    q_bars(sample_cnt,:) = q_bar; x_bars(sample_cnt,:) = x_bar;
    
    % Plot
    if DO_PLOT
        plot_qx_tilde_bar(chain_model_tilde,chain_model_bar,joi_tilde,joi_bar,...
            1,[0.0,0.6,0.6,0.4],[88,3],axis_info_ws+[0,0,0,3.0,0,0]); drawnow;
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
DO_PLOT = 1;

% Get the robot model
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);

% Sensitive analysis of revolute joints
[reverse_sensitiveness,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl);
joint_limits = get_joint_limits(chain_model,jnames_ctrl); % joint limits

% Sample feasble robot only
q_bars = zeros(n_sample,joint_limits.n);
tk = init_tk(sprintf('Collect [%d] samples',n_sample));
for sample_cnt = 1:n_sample % for desired # of samples
    tk = print_tk(tk,sample_cnt,n_sample);
    q_bar = joint_limits.min + joint_limits.range.*rand(1,joint_limits.n);
    chain_model = update_chain_q(chain_model,jnames_ctrl,q_bar,'IGNORE_LIMIT',0);
    chain_model_bar = fk_chain(chain_model);
    % Append
    q_bars(sample_cnt,:) = q_bar;
    % Plot
    if DO_PLOT
        xm = 0.15; ym = 0.15;
        plot_chain(chain_model_bar,'fig_idx',1,'subfig_idx',1,'fig_pos',[0.0,0.6,0.3,0.4],...
            'view_info',[88,16],'axis_info',axis_info_ws,...
            'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,...
            'PLOT_CAPSULE',0,...
            'mfc',[0.3,0.3,0.9],'mfa',0.6,'xm',xm,'ym',ym);
        drawnow;
    end
end

% Save
dt = datestr(now,'mmmm-dd-yyyy HH-MM-SS');
mat_path = sprintf('data/Domain of %s (%d) %s.mat',model_name,n_sample,dt);
save(mat_path,'q_bars');
fprintf(2,'[%s] saved.\n',mat_path);

% Done.
fprintf('Done.\n');

%% 3. Collect domain-specific MoCap data (x_tilde)
ccc

% Configuration
n_sample = 5e4; % number of samples to store
n_append_per_mocap = 20; % how many poses to append per mocap
DO_PLOT = 1;

% Get all bvh paths of CMU MoCap DB
mocap_infos = get_bvh_infos('mocap_folder','../../../cmu-mocap/');
m_list = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];
rng(0); r_idxs = randperm(length(mocap_infos)); m_list = unique([m_list,r_idxs(1:100)]);
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
        x_skeleton = [...
            get_uv(p_neck-p_root); ... % root to neck
            get_uv(p_rs-p_neck); ... % neck to right shoulder
            get_uv(p_re-p_rs); ... % right shoulder to right elbow
            get_uv(p_rh-p_re); ... % right elbow to right hand
            get_uv(p_ls-p_neck); ... % neck to left shoulder
            get_uv(p_le-p_ls); ... % left shoulder to left elbow
            get_uv(p_lh-p_le); ... % left elbow to left hand
            ]';
        
        % Append
        sample_cnt = sample_cnt + 1;
        x_tildes(sample_cnt,:) = x_skeleton;
        
        if DO_PLOT
            plot_chain(chain_mocap);
            plot_spheres([p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck]','sr',0.02);
            drawnow;
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
    [chains_mocap,joi_mocap,skeleton,time] = get_chains_mocap_with_cache(...
        m,'RE',0,'cache_folder','../../cache');
    L = length(chains_mocap);
    
    for tick = 1:L % for all tick
        chain_mocap = chains_mocap{tick};
        r2n_at_ease_rate = 0.9; % root to neck at-ease rate (0.0~1.0)
        [q,mr_vec] = run_rule_based_simple_motion_retargeting(...
            chain_mocap,joi_mocap,chain_model,joi_model,jnames_ctrl,...
            'r2n_at_ease_rate',r2n_at_ease_rate,'q_init','');
        chain_model = update_chain_q(chain_model,jnames_ctrl,q);
        chain_model = fk_chain(chain_model); % FK again
        SC = check_sc(chain_model);
        
        % Append paired data
        x_skeleton = get_skeleton_of_chain_mocap(chain_mocap,joi_mocap); % x in
        q_model = q; % q out 
        
        
        % Plot
        title_str = sprintf('[%s]-[%s] tick:[%d] SC:[%d]',m.subject,m.action,tick,SC);
        
        % Get JOI Positions of the Robot (pr: position of robot)
        [pr_root,pr_rs,pr_re,pr_rh,pr_ls,pr_le,pr_lh,pr_neck] = ...
            get_different_p_joi_robot_model(chain_model,joi_model);
        
        % Get JOI Positions of MoCap (pm: position of mocap)
        [pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck] = ...
            get_different_p_joi_mocap(chain_mocap,joi_mocap);
        [pm_neck_mr,pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr] = ...
            get_p_joi_motion_retarget(pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck,mr_vec);
        
        % Plot Robot model and workspaces
        axis_info = axis_info_ws;
        plot_chain(chain_model,'fig_idx',1,'subfig_idx',1,...
            'fig_pos',[0.0,0.3,0.5,0.6],'view_info',[85,14],'axis_info',axis_info,...
            'PLOT_MESH',1,'mfa',0.1,'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,...
            'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0,'PLOT_CAPSULE',0,...
            'title_str',title_str,'tfs',25);
        plot_cubes(repmat({p2t([0,0,0]')},1,4),...
            {ws.info{1}.min_vals,ws.info{2}.min_vals,ws.info{3}.min_vals,ws.info{4}.min_vals},...
            {ws.info{1}.len_vals,ws.info{2}.len_vals,ws.info{3}.len_vals,ws.info{4}.len_vals},...
            'colors',linspecer(4),'alpha',0.1);
        
        % Plot the JOI of the robot model
        joi_colors = linspecer(8); joi_sr = 0.02;
        plot_spheres([pr_rs,pr_re,pr_rh,pr_ls,pr_le,pr_lh,pr_neck]','subfig_idx',1,...
            'sr',joi_sr,'colors',joi_colors(2:end,:),'sfa',0.8);
        
        % Plot the Original MoCap Skeleton
        plot_chain(chain_mocap,'fig_idx',1,'subfig_idx',2,...
            'PLOT_LINK',1,'llc',[0.3,0.3,1.0],'llw',2,...
            'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0);
        
        % Plot the JOI of the Original MoCap Skeleton
        plot_T(p2t(pm_root),'subfig_idx',1,'PLOT_AXIS',1,'PLOT_SPHERE',0);
        plot_spheres([pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck]','subfig_idx',2,...
            'sr',joi_sr,'colors',joi_colors(2:end,:),'sfa',0.2);
        
        % Plot the JOI of the modified MoCap Skeleton
        plot_spheres([pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr,pm_neck_mr]',...
            'subfig_idx',3,'sr',joi_sr,'colors',joi_colors(2:end,:),'sfa',0.8);
        plot_lines([pm_root,pm_neck_mr,pm_rs_mr,pm_re_mr,pm_neck_mr,pm_ls_mr,pm_le_mr]',...
            [pm_neck_mr,pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr]',...
            'subfig_idx',1,'color','k','lw',3);
        drawnow;
        
    end % for tick = 1:L % for all tick
    
end % for m_idx = 1:length(mocap_idxs) % for all mocap data

% Done.
fprintf('Done.\n');

%% 5. Check collected paired data using a simple nearest neighbor (NN) search 
ccc

% Configuration
model_name = 'coman';

% Parse all paired data
dirs = dir(sprintf('data/Pairs of %s *',model_name));
x_tildes=[]; q_bars=[];
for i_idx = 1:length(dirs)
    mat_path = [dirs(i_idx).folder,'/',dirs(i_idx).name];
    l = load(mat_path);
    x_tildes = [x_tildes; l.x_tildes];
    q_bars = [q_bars; l.q_bars];
end

% Get the robot model
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);
[reverse_sensitiveness,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl);
joint_limits = get_joint_limits(chain_model,jnames_ctrl); % joint limits

% MoCap
mocap_infos = get_bvh_infos('mocap_folder','../../../cmu-mocap/');
m_list = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];
for m_idx = 1:length(m_list) % for different mocaps
    m = mocap_infos(m_list(m_idx));
    [chains_mocap,joi_mocap,skeleton,time] = get_chains_mocap_with_cache(...
        m,'RE',0,'cache_folder','../../cache');
    L = length(chains_mocap);
    
    for tick = 1:L % for each tick
        chain_mocap = chains_mocap{tick};
        % x_skeleton = get_skeleton_of_chain_mocap(chain_mocap,joi_mocap);
        [p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck] = ...
            get_different_p_joi_mocap(chain_mocap,joi_mocap);
        x_skeleton = [...
            get_uv(p_neck-p_root); ... % root to neck
            get_uv(p_rs-p_neck); ... % neck to right shoulder
            get_uv(p_re-p_rs); ... % right shoulder to right elbow
            get_uv(p_rh-p_re); ... % right elbow to right hand
            get_uv(p_ls-p_neck); ... % neck to left shoulder
            get_uv(p_le-p_ls); ... % left shoulder to left elbow
            get_uv(p_lh-p_le); ... % left elbow to left hand
            ]';
        % Motion retarget with S3LE
        [~,min_idx] = min(sum(abs(x_tildes-x_skeleton),2));
        q_star = q_bars(min_idx,:);
        
        % Update model 
        chain_model = update_chain_q(chain_model,jnames_ctrl,q_star,'IGNORE_LIMIT',0);
        chain_model = fk_chain(chain_model);
        
        % Plot
        plot_chain(chain_mocap,'subfig_idx',1,'PLOT_JOINT_AXIS',0);
        plot_chain(chain_model,'subfig_idx',2,'PLOT_JOINT_AXIS',0,'PLOT_CAPSULE',0,...
            'PLOT_ROTATE_AXIS',0);
        plot_spheres([p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck]','sr',0.02);
        drawnow;
        
    end % for tick = 1:L % for each tick
    
end % for m_idx = 1:length(m_list) % for different mocaps

%%








fprintf('Done.\n');


















