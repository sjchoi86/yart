addpath_code;
ccc;
%% Load results from EFT
ccc
addpath('../../../npy-matlab/npy-matlab');

% Configuration
model_name = 'coman';
npz_folder = '../../../3d_pose_with_eft/npz_results/';
dirs = dir([npz_folder, '*.npz']);

% Get the robot model
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);

for i_idx = 1:length(dirs) % for all npzs
    
    % Parse npz file that contains skeleton information from EFT
    npz_path = [dirs(i_idx).folder,'/',dirs(i_idx).name];
    a = unzip(npz_path,'archive'); % unzip first
    joint_traj = double(readNPY(a{1})); % first one is pos
    joint_traj = smoothdata(joint_traj,'sgolay'); % filter
    L = size(joint_traj,1);
    dur = double(readNPY(a{2})); % second one is duration
    fprintf('[%s] frame:[%d]~[%d] pos:[%dx%d].\n',...
        dirs(i_idx).name,dur(1),dur(2),L,size(joint_traj,2));
    
    % Parse mp4
    mp4_path = sprintf('../../../3d_pose_with_eft/videos/%s.mp4',...
        strtok(dirs(i_idx).name,'.'));
    V = VideoReader(mp4_path);
    HZ = 20; % everything is in 20HZ
    
    % Loop
    joi_mocap = get_joi_mocap('eft_skel');
    for tick = 1:L % for all ticks
        % Parse image frame and pos
        frame_idx = round(dur(1) + (dur(2)-dur(1))*(tick/L));
        frame = read(V,frame_idx);
        
        % Construct kinematic chain from points
        vec = reshape(joint_traj(tick,:),3,[])'; 
        [p_root,p_spine,p_neck,p_neck2,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_head,p_rp,p_lp] = ...
            parse_positions_from_eft_pos(vec);
        d_root2head = norm(p_root-p_spine)+norm(p_spine-p_neck)+norm(p_neck-p_head);
        vec = 1.0*vec/d_root2head;
        
        % Re-construct kinematic chain from points
        [p_root,p_spine,p_neck,p_neck2,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_head,p_rp,p_lp] = ...
            parse_positions_from_eft_pos(vec);
        
        % Make chain from the eft skeleton
        chain_mocap = init_chain('eft_skel');
        chain_mocap = add_joint_to_chain(chain_mocap,'root',p_root',[]);
        chain_mocap = add_joint_to_chain(chain_mocap,'spine',p_spine','root');
        chain_mocap = add_joint_to_chain(chain_mocap,'neck',p_neck','spine');
        chain_mocap = add_joint_to_chain(chain_mocap,'neck2',p_neck2','neck');
        chain_mocap = add_joint_to_chain(chain_mocap,'head',p_head','neck2');
        chain_mocap = add_joint_to_chain(chain_mocap,'rs',p_rs','neck');
        chain_mocap = add_joint_to_chain(chain_mocap,'re',p_re','rs');
        chain_mocap = add_joint_to_chain(chain_mocap,'rh',p_rh','re');
        chain_mocap = add_joint_to_chain(chain_mocap,'ls',p_ls','neck');
        chain_mocap = add_joint_to_chain(chain_mocap,'le',p_le','ls');
        chain_mocap = add_joint_to_chain(chain_mocap,'lh',p_lh','le');
        chain_mocap = add_joint_to_chain(chain_mocap,'rp',p_rp','root');
        chain_mocap = add_joint_to_chain(chain_mocap,'lp',p_lp','root');
        
        % Upfront chain
        chain_mocap = upfront_chain(chain_mocap,joi_mocap); % upfront mocap
        
        % Get skeleton feature
        x_skeleton = get_skeleton_of_chain_mocap(chain_mocap,joi_mocap);
        
        % Run optimization based motion retargeting
        r2n_at_ease_rate = 0.9; % root to neck at-ease rate (0.0~1.0)
        [q_tilde,mr_vec] = run_rule_based_simple_motion_retargeting(...
            chain_mocap,joi_mocap,chain_model,joi_model,jnames_ctrl,...
            'r2n_at_ease_rate',r2n_at_ease_rate,'q_init','','IGNORE_LIMIT',1);
        chain_model = update_chain_q(chain_model,jnames_ctrl,q_tilde);
        chain_model = fk_chain(chain_model); % FK again
        
        % Animate 
        title_str = sprintf('[%d/%d]',tick,L);
        axis_info = [-1,+1,-1,+1,-0.2,1.5];
        fig_pos1 = [0.0,0.6,0.4,0.3];
        fig_pos2 = [0.3,0.6,0.25,0.3];
        fig_pos3 = [0.5,0.6,0.25,0.3];
        fig1 = plot_img(frame,'fig_idx',1,'fig_pos',fig_pos1);
        fig2 = plot_chain(chain_mocap,'fig_idx',2,'fig_pos',fig_pos2,...
            'llw',4,'PLOT_JOINT_AXIS',0,'jsr',0.01,'PLOT_ROTATE_AXIS',0,...
            'PLOT_JOINT_SPHERE',0,...
            'title_str',title_str,'axis_info',axis_info,'view_info',[88,7]);
        colors = linspecer(joi_mocap.n);
        plot_joi_chain(chain_mocap,joi_mocap,'fig_idx',2,'PRINT_JOI_NAME',1,...
            'sr',0.035,'sfa',0.9,'colors',colors);
        fig3 = plot_chain(chain_model,'fig_idx',3,'fig_pos',fig_pos3,...
            'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,...
            'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,'PLOT_CAPSULE',0);
        drawnow;
        if ~ishandle(fig1) || ~ishandle(fig2), break; end
        
    end % for all ticks
    
end


%%
clc;ca




















































