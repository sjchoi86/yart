ccc
%% Self-Supervised Shared Latent Embedding (S3LE) of MoCap and Robot
ccc

% Configuration
model_name = 'coman'; % atlas / coman /

% Parse S3LE model
mat_path_x = 'script/nets/s3le_wae_x/weights.mat';
mat_path_q = 'script/nets/s3le_wae_y/weights.mat';
W_x = parse_wae(mat_path_x);
W_q = parse_wae(mat_path_q);

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
        z_from_x = encode_wae(W_x,x_skeleton);
        q_from_z = decode_wae(W_q,z_from_x);
        chain_model = update_chain_q(chain_model,jnames_ctrl,q_from_z,'IGNORE_LIMIT',0);
        chain_model = fk_chain(chain_model);
        
        % Plot
        plot_chain(chain_mocap,'subfig_idx',1);
        plot_chain(chain_model,'subfig_idx',2);
        plot_spheres([p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck]','sr',0.02);
        drawnow;
        
    end % for tick = 1:L % for each tick
    
end % for m_idx = 1:length(m_list) % for different mocaps

fprintf('Done.\n');

%%