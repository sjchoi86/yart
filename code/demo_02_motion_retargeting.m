ccc
%% 1. Minimal example of (rule-based) motion retargeting
ccc

% Configuration
ANIMATE = 0;

% Get the Robot Model
model_name = 'coman'; % atlas / baxter / coman / panda / sawyer
chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');
chain_model_sz = get_chain_sz(chain_model);
joi_model = get_joi_chain(chain_model);
ws = get_ws_with_cache(chain_model,joi_model,'ws_joi_types',{'rh','re','lh','le'},...
    'RE',0,'cache_folder','../cache','PLOT_EACH',0,'PLOT_FINAL',0);
joi_types = {'rh','lh'};
jnames_ctrl = get_jnames_ctrl_from_joi_types(chain_model,joi_model,joi_types,'EXCLUDE_TARGET',1);
n_ctrl = length(jnames_ctrl);
fprintf('[%s] n_ctrl:[%d].\n',model_name,n_ctrl);

% Get the MoCap Skeleton
mocap_infos = get_bvh_infos('mocap_folder','../../cmu-mocap/');
n_mocap = length(mocap_infos);
for m_idx = 1:50:n_mocap % for all mocap data
    m = mocap_infos(m_idx);
    bvh_path = m.full_path;  % bvh path
    [mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/');
    [skeleton,time] = load_raw_bvh(bvh_path);
    chains_mocap = get_chain_from_skeleton(skeleton,time,...
        'USE_METER',1,'ROOT_AT_ORIGIN',1,'Z_UP',1,'HZ',30);
    chains_mocap = chains_mocap(2:end); % exclude the first T-pose
    L = length(chains_mocap);
    if L > 300, L = 300; end % upperbond length
    chains_mocap = chains_mocap(1:L); % trim
    joi_mocap = get_joi_mocap(m.subject);
    fprintf('[%d/%d][%s]-[%s] description:[%s] L:[%d].\n',...
        m_idx,n_mocap,m.subject,m.action,action_str,L);
    
    % Cache path
    cache_path = sprintf('../cache/motion_retargeting/%s.mat',mocap_name);
    if exist(cache_path,'file')
        fprintf(2,'[%s] already exists. Skip this.\n',cache_path );
        continue;
    end
    
    % Loop
    if ANIMATE, ca; end 
    q_traj = zeros(L,n_ctrl);
    for tick = 1:L % for all tick
        chain_mocap = chains_mocap{tick};
        chain_mocap = upfront_chain(chain_mocap,joi_mocap); % upfront mocap skeleton
        
        % Get JOI Positions of MoCap (pm: position of mocap)
        [pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck] = ...
            get_different_p_joi_mocap(chain_mocap,joi_mocap);
        
        % Get lengthen rates of MoCap skeleton to match the robot morphology
        [r2n_rate,n2s_rate,s2e_rate,e2h_rate] = ...
            get_mocap_lengthen_rates(chain_model,joi_model,chain_mocap,joi_mocap);
        
        % Get Modified JOI Positions for Motion Retargeting (mr: motion retargeting)
        mr_vec = [0.5,0.0,0.0,0.0,r2n_rate,n2s_rate,s2e_rate,e2h_rate]; % rule-based MR
        [pm_neck_mr,pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr] = ...
            get_p_joi_motion_retarget(pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck,mr_vec);
        
        % Run IK
        ik = init_ik(chain_model,'joint_names_control',jnames_ctrl);
        ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'rh'),...
            'p',pm_rh_mr,'IK_P',1);
        ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'re'),...
            'p',pm_re_mr,'IK_P',1);
        ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'rs'),...
            'p',pm_rs_mr,'IK_P',1);
        ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'lh'),...
            'p',pm_lh_mr,'IK_P',1);
        ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'le'),...
            'p',pm_le_mr,'IK_P',1);
        ik = add_ik(ik,'joint_name',get_joint_name_from_joi_type(chain_model,joi_model,'ls'),...
            'p',pm_ls_mr,'IK_P',1);
        q = (ik.joint_limits_lower+ik.joint_limits_upper)/2; % median pose
        chain_model = update_chain_q(chain_model,ik.joint_names_control,q);
        chain_model = fk_chain(chain_model); % initialize chain
        while (ik.tick < 100) % loop until 500 ticks
            [ik,chain_model,q] = onestep_ik(ik,chain_model,q);
            [FLAG,ik,best_err] = check_ik_oscilating(ik); % check limbo
            if FLAG
                break;
            end
        end
        q_traj(tick,:) = q'; % append
        chain_model = update_chain_q(chain_model,jnames_ctrl,q);
        chain_model = fk_chain(chain_model); % initialize chain
        
        % Get JOI Positions of the Robot (pr: position of robot)
        [pr_root,pr_rs,pr_re,pr_rh,pr_ls,pr_le,pr_lh,pr_neck] = ...
            get_different_p_joi_robot_model(chain_model,joi_model);
        
        % Plot the robot model
        if ANIMATE
            title_str = sprintf('[%d/%d] [%s]-[%s]',tick,L,model_name,action_str);
            axis_info = [-1,1,-1.1,1.1,chain_model_sz.xyz_min(3),1.1];
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
            % plot_T(p2t(pm_root),'subfig_idx',1,'PLOT_AXIS',1,'PLOT_SPHERE',0);
            plot_spheres([pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck]','subfig_idx',2,...
                'sr',joi_sr,'colors',joi_colors(2:end,:),'sfa',0.2);
            % Plot the JOI of the modified MoCap Skeleton
            plot_spheres([pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr,pm_neck_mr]',...
                'subfig_idx',3,'sr',joi_sr,'colors',joi_colors(2:end,:),'sfa',0.8);
            plot_lines([pm_root,pm_neck_mr,pm_rs_mr,pm_re_mr,pm_neck_mr,pm_ls_mr,pm_le_mr]',...
                [pm_neck_mr,pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr]',...
                'subfig_idx',1,'color','k','lw',3);
            drawnow;
        end
        
    end % for tick = 2:L % for all tick
    
    % Save
    [p,~,~] = fileparts(cache_path);
    make_dir_if_not_exist(p);
    save(cache_path,'chain_model','jnames_ctrl','q_traj','m','chains_mocap','joi_mocap');
    fprintf(2,'[%s] saved.\n',cache_path);
    
end % for m_idx = 1:n_mocap % for all mocap data

%% 2. Check Cached Motion Retarget Results
ccc

% Get the Robot Model
model_name = 'coman'; % atlas / baxter / coman / panda / sawyer
chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');
chain_model_sz = get_chain_sz(chain_model);
joi_model = get_joi_chain(chain_model);

% Get the MoCap Skeleton
mocap_infos = get_bvh_infos('mocap_folder','../../cmu-mocap/');
n_mocap = length(mocap_infos);
for m_idx = 1:50:n_mocap % for all mocap data
    m = mocap_infos(m_idx);
    bvh_path = m.full_path;  % bvh path
    [mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/');
    
    % Load Cache
    cache_path = sprintf('../cache/motion_retargeting/%s.mat',mocap_name);
    if ~exist(cache_path,'file')
        fprintf(2,'[%s] does not exist.\n',cache_path );
        continue;
    else
        l = load(cache_path);
    end
    chain_model = l.chain_model; jnames_ctrl = l.jnames_ctrl; q_traj = l.q_traj;
    chains_mocap = l.chains_mocap; joi_mocap = l.joi_mocap;
    
    % Animate 
    L = size(q_traj,1);
    for tick = 1:L % for all tick
        q = q_traj(tick,:)';
        chain_model = update_chain_q(chain_model,jnames_ctrl,q);
        chain_model = fk_chain(chain_model);
        chain_mocap = chains_mocap{tick};
        chain_mocap = upfront_chain(chain_mocap,joi_mocap); % upfront mocap skeleton
        chain_mocap.joint(1).p(2) = -1; % move the mocap y to -1
        chain_mocap = fk_chain(chain_mocap);
        
        % Get JOI Positions of the Robot (pr: position of robot)
        [pr_root,pr_rs,pr_re,pr_rh,pr_ls,pr_le,pr_lh,pr_neck] = ...
            get_different_p_joi_robot_model(chain_model,joi_model);
        
        % Get JOI Positions of MoCap (pm: position of mocap)
        [pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck] = ...
            get_different_p_joi_mocap(chain_mocap,joi_mocap);
        
        % Plot the robot model
        title_str = sprintf('[%d/%d] [%s]-[%s]',tick,L,model_name,action_str);
        axis_info = [-1.2,1.2,-2.1,1.1,chain_model_sz.xyz_min(3),1.1];
        plot_chain(chain_model,'fig_idx',1,'subfig_idx',1,...
            'fig_pos',[0.0,0.3,0.6,0.6],'view_info',[85,5],'axis_info',axis_info,...
            'PLOT_MESH',1,'mfa',0.4,'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,...
            'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0,'PLOT_CAPSULE',0,...
            'title_str',title_str,'tfs',25);
        % Plot the JOI of the robot model
        joi_colors = linspecer(8); joi_sr = 0.02;
        plot_spheres([pr_rs,pr_re,pr_rh,pr_ls,pr_le,pr_lh,pr_neck]','subfig_idx',1,...
            'sr',joi_sr,'colors',joi_colors(2:end,:),'sfa',0.8);
        % Plot the Original MoCap Skeleton
        plot_chain(chain_mocap,'fig_idx',1,'subfig_idx',2,...
            'PLOT_LINK',1,'llc',[0.3,0.3,1.0],'llw',3,...
            'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0);
        % Plot the JOI of the Original MoCap Skeleton
        plot_spheres([pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck]','subfig_idx',2,...
            'sr',joi_sr,'colors',joi_colors(2:end,:),'sfa',0.7);
        drawnow limitrate;
        
    end % for tick = 1:L % for all tick
end % for m_idx = 1:n_mocap % for all mocap data

%%




