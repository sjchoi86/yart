addpath_code;
ccc

%% 1. Skeleton Representation
ccc

mocap_folder = '../../../cmu-mocap/';
mocap_infos = get_bvh_infos('mocap_folder',mocap_folder);
n_mocap = length(mocap_infos);
fprintf('We have [%d] mocaps.\n',n_mocap);

rng(4); r_idxs = randperm(n_mocap);
m_idxs = r_idxs(1:4);

for i_idx = 1:length(m_idxs)
    % Load MoCap
    m = mocap_infos(m_idxs(i_idx));
    [mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder',mocap_folder);
    [skeleton,time] = load_raw_bvh(m.full_path);
    joi_mocap = get_joi_mocap(m.subject); 
    chains_mocap = get_chain_from_skeleton(skeleton,time,...
        'USE_METER',1,'ROOT_AT_ORIGIN',1,'Z_UP',1,'HZ',30);
    
    % Select one skeleton and plot
    chain_mocap = chains_mocap{2}; % select
    chain_mocap = upfront_chain(chain_mocap,joi_mocap); % upfront
    llm_vec = 1.0+0.2*rand(1,5);
    llm_stuct = struct('rh',llm_vec(1),'re',llm_vec(2),'rs',llm_vec(3),...
        'lh',llm_vec(1),'le',llm_vec(2),'ls',llm_vec(3),...
        'spine1',llm_vec(4),'neck',llm_vec(5));
    chain_mocap = get_chain_mocap_llm(chain_mocap,joi_mocap,llm_stuct); % llm
    chain_mocap.joint(1).p(2) = i_idx-1; chain_mocap = fk_chain(chain_mocap); % move
    
    axis_info = [-0.9,+0.5,-0.5,4,-1.2,0.65]; view_info = [88,4];
    plot_chain(chain_mocap,...
        'fig_idx',1,'subfig_idx',i_idx,'fig_pos',[0.0,0.6,0.6,0.4],'axis_info',axis_info,...
        'PLOT_LINK',1,'llw',2,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,'jal',0.02,...
        'PLOT_JOINT_SPHERE',1,'jsfc','k','jsr',0.02,'jsfa',0.9,'PRINT_JOINT_NAME',0,...
        'title_str','','tfs',16,'view_info',view_info,'GRID_ON',0);
    plot_joi_chain(chain_mocap,joi_mocap, ...
        'fig_idx',1,'subfig_idx',i_idx,'sr',0.04,'sfa',0.5,'colors',linspecer(joi_mocap.n),...
        'PRINT_JOI_NAME',0);
    drawnow;
end


%% 2. Illustrative Figure for Data Generating Processes
ccc

% Get robot model
model_name = 'coman';
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
n_joint = length(jnames_ctrl);
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);

% Update robot model (relaxed model)
joint_limits = get_joint_limits(chain_model,jnames_ctrl); % joint limits
q_min = joint_limits.min; q_max = joint_limits.max; q_range = joint_limits.range;
% q_tilde = [0,0,0,0,0,0,0,0,0,0,0,0,0]*D2R;
rng(9);
relax_rate = 1.5;
q_tilde = q_min - (relax_rate-1)/2*q_range + relax_rate*q_range.*rand(1,joint_limits.n);
chain_model = update_chain_q(chain_model,jnames_ctrl,q_tilde,'IGNORE_LIMIT',1);
chain_model_tilde = fk_chain(chain_model);
[p_root_tilde,p_rs_tilde,p_re_tilde,p_rh_tilde,p_ls_tilde,p_le_tilde,p_lh_tilde,...
    p_neck_tilde] = ...
    get_different_p_joi_robot_model(chain_model_tilde,joi_model);
joi_tilde = [p_root_tilde,p_rs_tilde,p_re_tilde,p_rh_tilde,p_ls_tilde,p_le_tilde,p_lh_tilde,...
    p_neck_tilde]';

% Corresponding Skeleton (relaxed skeleton)
x_tilde = get_skeleton_of_chain_model(chain_model_tilde,joi_model);

% Feasible robot model
q_tpose = zeros(1,n_joint);
q_bar = min(max(q_tilde,q_min),q_max);
chain_model = update_chain_q(chain_model,jnames_ctrl,q_bar,'IGNORE_LIMIT',0);
chain_model_bar = fk_chain(chain_model);
[SC,sc_ij_list] = check_sc(chain_model_bar); % check self-collision
MAKE_Q_BAR_COLLISION_FREE = 1; sc_loop_cnt = 0;
while (SC && MAKE_Q_BAR_COLLISION_FREE) % until collision-free
    sc_loop_cnt = sc_loop_cnt + 1;
    sc_link_idxs = unique(sc_ij_list(:));
    joint_names = [];
    for l_idx = 1:length(sc_link_idxs)
        sc_link_idx = sc_link_idxs(l_idx);
        joint_idx = chain_model.link(sc_link_idx).joint_idx;
        joint_name = chain_model.joint_names{joint_idx};
        idx_route = get_idx_route(chain_model,joint_name);
        joint_names = [joint_names,chain_model.joint_names(idx_route)];
    end
    joint_names = unique(joint_names); % joints to control to make collision-free
    [joint_names,~,match_idx] = intersect(joint_names,jnames_ctrl);
    % Weighted average with T-pose to make it collision-free
    mix_ratio = 0.95;
    q_bar(match_idx) = mix_ratio*q_bar(match_idx) + (1-mix_ratio)*q_tpose(match_idx);
    chain_model_bar = update_chain_q(chain_model_bar,jnames_ctrl,q_bar,'IGNORE_LIMIT',0);
    chain_model_bar = fk_chain(chain_model_bar);
    [SC,sc_ij_list] = check_sc(chain_model_bar); % check self-collision
end

% Feasible skeleton
[p_root_bar,p_rs_bar,p_re_bar,p_rh_bar,p_ls_bar,p_le_bar,p_lh_bar,p_neck_bar] = ...
    get_different_p_joi_robot_model(chain_model_bar,joi_model);
joi_bar = [p_root_bar,p_rs_bar,p_re_bar,p_rh_bar,p_ls_bar,p_le_bar,p_lh_bar,p_neck_bar]';
x_bar = get_skeleton_of_chain_model(chain_model_bar,joi_model);

% Plot
view_info = [68,21];
axis_info = [-0.45,0.45,-0.45,0.45,-0.5168,0.7]; % axis_info_ws;
xm = 0.2; ym = 0.15;
mfa = 0.9; color_tilde = [0.9,0.4,0.4];
% q_tilde
title_str = '';
plot_chain(chain_model_tilde,'fig_idx',1,'subfig_idx',1,'fig_pos',[0.0,0.5,0.15,0.3],...
    'view_info',view_info,'axis_info',axis_info,...
    'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,...
    'PLOT_CAPSULE',0,'mfc',color_tilde,'mfa',mfa,'xm',xm,'ym',ym,...
    'title_str',title_str);
% q_bar
title_str = '';
plot_chain(chain_model_bar,'fig_idx',2,'subfig_idx',1,'fig_pos',[0.15,0.5,0.15,0.3],...
    'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,...
    'PLOT_CAPSULE',0,'view_info',view_info,'axis_info',axis_info,...
    'mfc',0.4*[1,1,1],'mfa',mfa,'xm',xm,'ym',ym,'title_str',title_str);

% x_tlide
title_str = '';
sr = chain_model_tilde.xyz_len(3)/30;
set_fig_position(figure(3),'position',[0.0,0.15,0.15,0.3],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',view_info,'axis_info',axis_info,'SET_DRAGZOOM',1,'GRID_ON',1,'xm',xm,'ym',ym,...
    'title_str',title_str);
plot_spheres(joi_tilde,'fig_idx',3,'subfig_idx',1,...
    'sr',sr,'sfc',color_tilde,'sfa',0.7);
plot_skeleton_from_joi(joi_tilde,'fig_idx',3,'subfig_idx',1,...
    'color',color_tilde,'lw',3);

% x_bar
title_str = '';
sr = chain_model_tilde.xyz_len(3)/30;
set_fig_position(figure(4),'position',[0.15,0.15,0.15,0.3],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',view_info,'axis_info',axis_info,'SET_DRAGZOOM',1,'GRID_ON',1,'xm',xm,'ym',ym,...
    'title_str',title_str);
plot_spheres(joi_bar,'fig_idx',4,'subfig_idx',2,'sr',sr,'sfc',0.3*[1,1,1],'sfa',0.7);
plot_skeleton_from_joi(joi_bar,'fig_idx',4,'subfig_idx',2,'color',0.3*[1,1,1],'lw',3);

%% 3. Domain specific data generation
ccc

% 1. Domain-specific model
rng(2); % fix seed 
model_name = 'coman'; % atlas / coman /
[chain_model,chain_model_sz,joi_model,ws,jnames_ctrl] = ...
    get_robot_model_information_for_motion_retargeting(...
    model_name,'RE',0,'cache_folder','../../cache','urdf_folder','../../urdf');
axis_info_ws = get_axis_info_from_chain(ws,chain_model_sz,'margin_rate',0.3);
[reverse_sensitiveness,jnames_ctrl] = get_reverse_sensitiveness(...
    chain_model,joi_model,jnames_ctrl);
joint_limits = get_joint_limits(chain_model,jnames_ctrl); % joint limits
q_bar = joint_limits.min + joint_limits.range.*rand(1,joint_limits.n);
chain_model = update_chain_q(chain_model,jnames_ctrl,q_bar,'IGNORE_LIMIT',0);
chain_model_bar = fk_chain(chain_model);
xm = 0.2; ym = 0.15;
plot_chain(chain_model_bar,'fig_idx',1,'subfig_idx',1,'fig_pos',[0.0,0.6,0.2,0.4],...
    'view_info',[78,16],'axis_info','',...
    'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,...
    'PLOT_CAPSULE',0,...
    'mfc',[0.3,0.3,0.9],'mfa',0.6,'xm',xm,'ym',ym);
drawnow;

% 2. Domain-specific skeleton
rng(2); % fix seed
mocap_infos = get_bvh_infos('mocap_folder','../../../cmu-mocap/');
r_idxs = randperm(length(mocap_infos));
m = mocap_infos(r_idxs(1));
[chains_mocap,joi_mocap,skeleton,time] = get_chains_mocap_with_cache(...
    m,'RE',0,'cache_folder','../../cache');
L = length(chains_mocap);
r_idxs = randperm(L);
chain_mocap = chains_mocap{r_idxs(1)}; % skeleton chain
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
plot_chain(chain_mocap,'fig_idx',2,'fig_pos',[0.2,0.6,0.2,0.4],'view_info',[70,26],...
    'PLOT_JOINT_AXIS',0,'llc',[0.3,0.3,0.9]);
plot_spheres([p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck]','fig_idx',2,...
    'sfc',[0.3,0.3,0.9],'sr',0.04);
drawnow;


%%



