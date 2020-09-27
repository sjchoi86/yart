%%
% 1. Minimal example of (rule-based) motion retargeting
% 2. Check Cached Simple Motion Retarget Results
% 3.
%
ccc

%% 1. Minimal example of (rule-based) motion retargeting
ccc

% Configuration
ANIMATE = 1;
SKIP_IF_EXISTS = 0;
model_names = {'social_robot'}; % {'social_robot','atlas','baxter','coman'};
r2n_at_ease_rate = 0.95; % root to neck at-ease rate (0.0~1.0)

% Parse mocap infos
mocap_infos = get_bvh_infos('mocap_folder','../../cmu-mocap/');
mocap_idxs = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];

mocap_idxs = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488];

for i_idx = 1:length(model_names) % for all robot models
    % Get the Robot Model
    model_name = model_names{i_idx};
    [chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
        get_robot_model_information_for_motion_retargeting(model_name,'RE',0);
    n_ctrl = length(jnames_ctrl); % number of joints to control
    for j_idx = 1:length(mocap_idxs) % for all mocap data
        % Get the MoCap Skeleton
        mocap_idx = mocap_idxs(j_idx);
        m = mocap_infos(mocap_idx);
        [mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/');
        [skeleton,time] = load_raw_bvh(m.full_path);
        chains_mocap = get_chain_from_skeleton(skeleton,time,...
            'USE_METER',1,'ROOT_AT_ORIGIN',1,'Z_UP',1,'HZ',30);
        chains_mocap = chains_mocap(2:end); % exclude the first T-pose of the skeleton
        L = length(chains_mocap);
        if L > 600, L = 600; end % upperbound length
        chains_mocap = chains_mocap(1:L); % trim
        joi_mocap = get_joi_mocap(m.subject);
        fprintf('[%d/%d][%d/%d] [%s]-[%s] description:[%s] L:[%d].\n',...
            i_idx,length(model_names),j_idx,length(mocap_idxs),m.subject,m.action,action_str,L);
        
        % Cache path
        cache_path = sprintf('../cache/simple_motion_retargeting/%s_%s.mat',model_name,mocap_name);
        if exist(cache_path,'file') && SKIP_IF_EXISTS
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
            if tick == 1
                q_init = '';
            end
            [q,mr_vec] = run_rule_based_simple_motion_retargeting(...
                chain_mocap,joi_mocap,chain_model,joi_model,jnames_ctrl,...
                'r2n_at_ease_rate',r2n_at_ease_rate,'q_init',q_init);
            q_init = q; % start IK from the previous q
            
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
        end % for tick = 1:L % for all tick
        
        % Get collision-free joint trajectory
        q_traj_cf = get_q_traj_cf(chain_model,jnames_ctrl,q_traj,'TEMPORAL_CONSISTENCY',0);
        % q_traj_cf = smoothdata(q_traj_cf,'movmean',3); % smooth
        
        % Save the motion retarget results 'q_traj' and 'q_traj_cf'
        [p,~,~] = fileparts(cache_path);
        make_dir_if_not_exist(p);
        save(cache_path,'chain_model','jnames_ctrl','q_traj','q_traj_cf',...
            'm','chains_mocap','joi_mocap');
        fprintf(2,'[%s] saved.\n',cache_path);
        
    end % for j_idx = 1:n_mocap % for all mocap data
end % for i_idx = 1:length(model_names) % for all robot models


%% 2. Check Cached Simple Motion Retarget Results
ccc

% Configuration
SAVE_VID = 1;

% Get the Robot Model
model_name = 'social_robot'; % atlas / baxter / coman / social_robot
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(model_name);

% Get the MoCap Skeleton
mocap_infos = get_bvh_infos('mocap_folder','../../cmu-mocap/');
n_mocap = length(mocap_infos);
mocap_idxs = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488,...
    1104,1116,1118,1123,1124,1125,1126,1127,1192,1208,1920];

mocap_idxs = [123,126,433,435,436,437,438,439,440,441,442,443,444,480,481,488];

for i_idx = 1:length(mocap_idxs) % for different mocap data
    m = mocap_infos(mocap_idxs(i_idx)); % mocap info
    bvh_path = m.full_path;  % bvh path
    [mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/'); % mocap name
    
    % Load Cached Motion Retarget Results
    cache_path = sprintf('../cache/simple_motion_retargeting/%s_%s.mat',model_name,mocap_name);
    if ~exist(cache_path,'file')
        fprintf(2,'[%s] does not exist.\n',cache_path );
        continue;
    end
    l = load(cache_path); % load cached results
    chain_model = l.chain_model; jnames_ctrl = l.jnames_ctrl;
    q_traj_raw = l.q_traj; q_traj_cf = l.q_traj_cf;
    chains_mocap = l.chains_mocap; joi_mocap = l.joi_mocap;
    
    % Extra smoothing
    q_traj_raw = smoothdata(q_traj_raw,'movmean',3);
    q_traj_cf = smoothdata(q_traj_cf,'movmean',3);
    
    % Save to Video
    vid_path = sprintf('../vid/simple_motion_retargeting/%s_%s.mp4',model_name,mocap_name);
    vid_obj = init_vid_record(vid_path,'HZ',30,'SAVE_VID',SAVE_VID);
    
    % Animate
    L = size(q_traj_raw,1);
    for tick = 1:L % for all tick
        q_raw = q_traj_raw(tick,:)';
        q_cf = q_traj_cf(tick,:)'; % <= use collision-free pose
        chain_model_raw = update_chain_q_fk(chain_model,jnames_ctrl,q_raw);
        chain_model_cf = update_chain_q_fk(chain_model,jnames_ctrl,q_cf);
        SC = check_sc(chain_model_cf); % check SC
        chain_mocap = chains_mocap{tick};
        chain_mocap = upfront_chain(chain_mocap,joi_mocap); % upfront mocap skeleton
        chain_mocap.joint(1).p(2) = -1.0; % move the mocap y to -1.0m
        chain_mocap = fk_chain(chain_mocap);
        title_str = sprintf('[%d/%d] [%s]-[%s] SC:[%d]',tick,L,model_name,action_str,SC);
        plot_simple_rule_based_motion_retargeted_result2(...
            chain_model_raw,chain_model_cf,joi_model,chain_model_sz,chain_mocap,joi_mocap,title_str);
        if SAVE_VID, drawnow; record_vid(vid_obj); % save each frame
        else, drawnow; % limitrate;
        end
    end % for tick = 1:L % for all tick
    end_vid_record(vid_obj); % finalize video saving
    
end % for i_idx = 1:length(mocap_idxs) % for different mocap data

%%
