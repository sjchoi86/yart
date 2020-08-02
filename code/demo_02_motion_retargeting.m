ccc
%% 1. Minimal example of (rule-based) motion retargeting
ccc

% Configuration
ANIMATE = 0;

% Get the Robot Model
model_name = 'coman'; % atlas / baxter / coman / panda / sawyer
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(model_name);
n_ctrl = length(jnames_ctrl); % number of joints to control
fprintf('[%s] n_ctrl:[%d].\n',model_name,n_ctrl);

% Get the MoCap Skeleton
mocap_infos = get_bvh_infos('mocap_folder','../../cmu-mocap/');
n_mocap = length(mocap_infos);
mocap_idxs = [109,123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];
for m_idx = mocap_idxs % for all mocap data
    m = mocap_infos(m_idx);
    [mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/');
    [skeleton,time] = load_raw_bvh(m.full_path);
    chains_mocap = get_chain_from_skeleton(skeleton,time,...
        'USE_METER',1,'ROOT_AT_ORIGIN',1,'Z_UP',1,'HZ',30);
    chains_mocap = chains_mocap(2:end); % exclude the first T-pose of the skeleton
    L = length(chains_mocap);
    if L > 300, L = 300; end % upperbound length
    chains_mocap = chains_mocap(1:L); % trim
    joi_mocap = get_joi_mocap(m.subject);
    fprintf('[%d/%d][%s]-[%s] description:[%s] L:[%d].\n',...
        m_idx,n_mocap,m.subject,m.action,action_str,L);
    
    % Cache path
    cache_path = sprintf('../cache/motion_retargeting/%s_%s.mat',model_name,mocap_name);
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
        
        % Run a simple rule-based motion retargeting
        [q,mr_vec] = run_rule_based_simple_motion_retargeting(...
            chain_mocap,joi_mocap,chain_model,joi_model,jnames_ctrl);
        q_traj(tick,:) = q'; % append motion retargeted joint angles
        chain_model = update_chain_q(chain_model,jnames_ctrl,q);
        chain_model = fk_chain(chain_model); % FK again
        
        % Plot the robot model
        if ANIMATE
            title_str = sprintf('[%d/%d] [%s]-[%s]',tick,L,model_name,action_str);
            plot_simple_rule_based_motion_retargeting(...
                chain_model,chain_model_sz,joi_model,ws,chain_mocap,joi_mocap,mr_vec,title_str);
            drawnow;
        end
        
    end % for tick = 2:L % for all tick
    
    % Save the motion retarget results 'q_traj'
    [p,~,~] = fileparts(cache_path);
    make_dir_if_not_exist(p);
    save(cache_path,'chain_model','jnames_ctrl','q_traj','m','chains_mocap','joi_mocap');
    fprintf(2,'[%s] saved.\n',cache_path);
    
end % for m_idx = 1:n_mocap % for all mocap data

%% 2. Check Cached Simple Motion Retarget Results
ccc

% Get the Robot Model
model_name = 'coman'; % atlas / baxter / coman / panda / sawyer
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(model_name);

% Get the MoCap Skeleton
mocap_infos = get_bvh_infos('mocap_folder','../../cmu-mocap/');
n_mocap = length(mocap_infos);
mocap_idxs = [109,123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];
for m_idx = mocap_idxs
    m = mocap_infos(m_idx);
    bvh_path = m.full_path;  % bvh path
    [mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/');
    
    % Load Cached Motion Retarget Results
    cache_path = sprintf('../cache/motion_retargeting/%s_%s.mat',model_name,mocap_name);
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
        chain_mocap.joint(1).p(2) = -1.0; % move the mocap y to -1.0m
        chain_mocap = fk_chain(chain_mocap);
        title_str = sprintf('[%d/%d] [%s]-[%s]',tick,L,model_name,action_str);
        plot_simple_rule_based_motion_retargeted_result(...
            chain_model,joi_model,chain_model_sz,chain_mocap,joi_mocap,title_str);
        drawnow limitrate;
        
    end % for tick = 1:L % for all tick
end % for m_idx = 1:n_mocap % for all mocap data

%%




