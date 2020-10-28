addpath_code
ccc
%% Self-Supervised Shared Latent Embedding (S3LE) of MoCap and Robot
ccc

% Configuration
model_name = 'coman'; % atlas / coman /

% Parse robot specific data (q_bars)
USE_SSL_PAIRS = 1;
USE_MR_PAIRS  = 1;

% Get data from SS sampling 
x_tildes_ssl = []; q_bars_ssl = [];
if USE_SSL_PAIRS
    dirs = dir(sprintf('data/Pairs of %s *',model_name));
    for i_idx = 1:length(dirs) % for i_idx = 1:length(dirs) % for all mat files
        l = load([dirs(i_idx).folder,'/',dirs(i_idx).name]);
        gammas = [0.0,0.2,0.5];
        for gamma = gammas
            x_tildes_ssl = [x_tildes_ssl; gamma*l.x_bars + (1-gamma)*l.x_tildes];
            q_bars_ssl = [q_bars_ssl; l.q_bars];
        end
    end % for i_idx = 1:length(dirs) % for all mat files
end
fprintf('SSL pairs [%d] data.\n',size(x_tildes_ssl,1));

% Get data from motion retargeting
x_tildes_mr_aug = []; q_bars_mr_aug = [];
x_tildes_mr = []; q_bars_mr = [];
if USE_MR_PAIRS
    dirs = dir(sprintf('data/Glue of CMU mocap*%s*',model_name));
    dirs = dirs(1:100);
    % fprintf(' [%d] mocaps. \n',length(dirs));
    for i_idx = 1:length(dirs) % for all mat files
        l = load([dirs(i_idx).folder,'/',dirs(i_idx).name]);
        gammas = [0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7];
        for gamma = gammas
            x_tildes_mr_aug = [x_tildes_mr_aug; gamma*l.x_bars + (1-gamma)*l.x_tildes];
            q_bars_mr_aug = [q_bars_mr_aug; l.q_bars];
        end
        x_tildes_mr = [x_tildes_mr; l.x_tildes];
        q_bars_mr = [q_bars_mr; l.q_bars];
    end % for i_idx = 1:length(dirs) % for all mat files
end
fprintf('MR pairs [%d] data.\n',size(x_tildes_mr_aug,1));

% Get Robot specific
q_bars_domain = [];
dirs = dir(sprintf('data/Domain of %s*',model_name));
for i_idx = 1:length(dirs) % for all mat files
    l = load([dirs(i_idx).folder,'/',dirs(i_idx).name]);
    q_bars_domain = [q_bars_domain; l.q_bars];
end % for i_idx = 1:length(dirs) % for all mat files
fprintf('Robot-specific [%d] data.\n',size(q_bars_domain,1));

%%
clc; ca;

% Set nonparametric data
USE_SSL_AND_MR = 1;
if USE_SSL_AND_MR
    q_data_s3le = [q_bars_ssl; q_bars_mr_aug; q_bars_domain]; 
else
    q_data_s3le = [q_bars_mr_aug; q_bars_domain];
end

x_data_lwl2 = x_tildes_mr;
q_data_lwl2 = [q_bars_mr; q_bars_domain];

% Parse S3LE model
mat_path_x = 'script/nets/s3le_wae_x/weights.mat';
mat_path_q = 'script/nets/s3le_wae_y/weights.mat';
S3LE_W_x = parse_wae(mat_path_x);
S3LE_W_q = parse_wae(mat_path_q);
fprintf('WAEs parsed. x_it:[%d] q_it:[%d].\n',S3LE_W_x.it,S3LE_W_q.it);
z_from_q_s3le = encode_wae(S3LE_W_q,q_data_s3le); % encode
fprintf('Total [%d] data for S3LE.\n',size(z_from_q_s3le,1));

% Parse LwL2 model
mat_path_x = 'script/nets/lwl2_wae_x/weights.mat';
mat_path_q = 'script/nets/lwl2_wae_y/weights.mat';
LWL2_W_x = parse_wae(mat_path_x);
LWL2_W_q = parse_wae(mat_path_q);
fprintf('WAEs parsed. x_it:[%d] q_it:[%d].\n',LWL2_W_x.it,LWL2_W_q.it);
z_from_q_lwl2 = encode_wae(LWL2_W_q,q_data_lwl2); % encode
fprintf('Total [%d] data for LWL2.\n',size(z_from_q_lwl2,1));


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

% Loop
for m_idx = 1:length(m_list) % for different mocaps
    m = mocap_infos(m_list(m_idx));
    [chains_mocap,joi_mocap,skeleton,time] = get_chains_mocap_with_cache(...
        m,'RE',0,'cache_folder','../../cache');
    L = length(chains_mocap);
    
    for tick = 1:L % for each tick
        chain_mocap = chains_mocap{tick};
        
        % Get skeleton feature
        x_skeleton = get_skeleton_of_chain_mocap(chain_mocap,joi_mocap);
        
        % 1. Motion retarget with S3LE
        z_from_x = encode_wae(S3LE_W_x,x_skeleton);
        q_from_z_s3le = decode_wae(S3LE_W_q,z_from_x); % decoder-predicted pose
        [~,min_idx] = min(sum(abs(z_from_q_s3le - z_from_x),2));
        q_np = q_data_s3le(min_idx,:); % nonparametric regression
        q = q_np; % raw S3LE input
        if (tick == 1) 
        else
            % update with step restriction 
            q_diff = q - q_prev_s3le; % diff
            q_diff_max = 50*D2R;
            q_diff_min = -20*D2R;
            q_diff(q_diff>q_diff_max) = q_diff_max;
            q_diff(q_diff<q_diff_min) = q_diff_min;
            q_step_restrict = q_prev_s3le + q_diff; 
            % EMA smoothing
            alphas = [0.8,0.5,0.2,0.0]; % 1.0: fully update / 0: no update
            for alpha = alphas
                q_check = alpha*q_step_restrict + (1-alpha)*q_prev_s3le; 
                chain_model_check = update_chain_q(chain_model,jnames_ctrl,q_check,...
                    'IGNORE_LIMIT',0);
                chain_model_check = fk_chain(chain_model_check);
                SC = check_sc(chain_model_check);
                if SC == 0
                    q = q_check;
                    break;
                end
            end
        end
        % q_s3le = q_from_z_s3le;
        q_s3le = q;
        
        
        % 2. Motion retarget with LWL2 (Baseline)
        z_from_x = encode_wae(LWL2_W_x,x_skeleton);
        q_from_z_lwl2 = decode_wae(LWL2_W_q,z_from_x); % decoder-predicted pose
        [~,min_idx] = min(sum(abs(z_from_q_lwl2 - z_from_x),2));
        q_np = q_data_lwl2(min_idx,:); % nonparametric regression
        q = q_np; % raw S3LE input
        if (tick == 1) 
        else
            % update with step restriction 
            q_diff = q - q_prev_lwl2; % diff
            q_diff_max = 50*D2R;
            q_diff_min = -20*D2R;
            q_diff(q_diff>q_diff_max) = q_diff_max;
            q_diff(q_diff<q_diff_min) = q_diff_min;
            q_step_restrict = q_prev_lwl2 + q_diff; 
            % EMA smoothing
            alphas = [0.8,0.5,0.2,0.0]; % 1.0: fully update / 0: no update
            for alpha = alphas
                q_check = alpha*q_step_restrict + (1-alpha)*q_prev_lwl2; 
                chain_model_check = update_chain_q(chain_model,jnames_ctrl,q_check,...
                    'IGNORE_LIMIT',0);
                chain_model_check = fk_chain(chain_model_check);
                SC = check_sc(chain_model_check);
                if SC == 0
                    q = q_check;
                    break;
                end
            end
        end
        q_lwl2 = q;
        
        % Update model with S3LE
        chain_model_s3le = update_chain_q(chain_model,jnames_ctrl,q_s3le,'IGNORE_LIMIT',0);
        q_prev_s3le = q_s3le; % backup
        chain_model_s3le = fk_chain(chain_model_s3le);
        SC_s3le = check_sc(chain_model_s3le);
        
        % Update model with LWL2
        chain_model_lwl2 = update_chain_q(chain_model,jnames_ctrl,q_lwl2,'IGNORE_LIMIT',0);
        q_prev_lwl2 = q_lwl2; % backup
        chain_model_lwl2 = fk_chain(chain_model_lwl2);
        SC_lwl2 = check_sc(chain_model_lwl2);
        
        % Plot
        axis_info = axis_info_ws + [0,0,-1,0,0,0];
        title_str = sprintf('[%d/%d] SC S3LE:[%d] LWL2:[%d]',...
            tick,L,SC_s3le,SC_lwl2);
        chain_mocap.joint(1).p(2) = -1;
        chain_mocap = fk_chain(chain_mocap);
        plot_chain(chain_mocap,'subfig_idx',1,'fig_pos',[0.0,0.5,0.5,0.45],...
            'PLOT_JOINT_AXIS',0,'title_str',title_str,'axis_info',axis_info);
        plot_chain(chain_model_s3le,'subfig_idx',2,...
            'PLOT_CAPSULE',0,'PLOT_JOINT_AXIS',0,'PLOT_ROTATE_AXIS',0,'title_str','');
        plot_chain(chain_model_lwl2,'subfig_idx',3,...
            'mfc',[0.9,0.4,0.4],...
            'PLOT_CAPSULE',0,'PLOT_JOINT_AXIS',0,'PLOT_ROTATE_AXIS',0,'title_str','');
        [p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck] = ...
            get_different_p_joi_mocap(chain_mocap,joi_mocap);
        plot_spheres([p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck]','sr',0.02);
        drawnow;
        
    end % for tick = 1:L % for each tick
    
end % for m_idx = 1:length(m_list) % for different mocaps

fprintf('Done.\n');

%%

