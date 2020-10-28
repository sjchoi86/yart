function plot_paired_data_from_rule_based_motion_retarget(...
    chain_model,joi_model,chain_mocap,joi_mocap,mr_vec,...
    x_tilde,x_bar,axis_info_ws,title_str)
%
% Plot
% 1. robot model and retargeting results
% 2. relaxed skeleton and feasible skeleton
%

fig_pos1 = [0.0,0.6,0.3,0.4];
fig_pos2 = [0.3,0.6,0.3,0.4];
view_info = [85,14];

%% Get JOI Positions of the Robot (pr: position of robot)
[pr_root,pr_rs,pr_re,pr_rh,pr_ls,pr_le,pr_lh,pr_neck] = ...
    get_different_p_joi_robot_model(chain_model,joi_model);

% Get JOI Positions of MoCap (pm: position of mocap)
[pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck] = ...
    get_different_p_joi_mocap(chain_mocap,joi_mocap);
[pm_neck_mr,pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr] = ...
    get_p_joi_motion_retarget(pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck,mr_vec);

%% Figure 1. Plot Robot model and workspaces

axis_info = axis_info_ws;
plot_chain(chain_model,'fig_idx',1,'subfig_idx',1,...
    'fig_pos',fig_pos1,'view_info',view_info,'axis_info',axis_info,...
    'PLOT_MESH',1,'mfa',0.1,'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,...
    'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0,'PLOT_CAPSULE',0,...
    'title_str',title_str,'tfs',25);
if 0 % workspace
    plot_cubes(repmat({p2t([0,0,0]')},1,4),...
        {ws.info{1}.min_vals,ws.info{2}.min_vals,ws.info{3}.min_vals,ws.info{4}.min_vals},...
        {ws.info{1}.len_vals,ws.info{2}.len_vals,ws.info{3}.len_vals,ws.info{4}.len_vals},...
        'colors',linspecer(4),'alpha',0.1);
end

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

% Plot mocap root and neck
p_root = get_p_joi_type(chain_mocap,joi_mocap,'root');
p_neck = get_p_joi_type(chain_mocap,joi_mocap,'neck');
plot_T(p2t(p_root),'subfig_idx',1,'PLOT_AXIS',0,'sr',0.02,'sfc','r');
plot_T(p2t(p_neck),'subfig_idx',2,'PLOT_AXIS',0,'sr',0.02,'sfc','b');
drawnow;


%% Figure 2. Plot skeletons from features
[d_r2n,d_n2s,d_s2e,d_e2h] = get_link_lengths_chain(chain_model,joi_model);
plot_skeleton_from_features(x_tilde,'fig_idx',2,'subfig_idx',1,...
    'd_r2n',d_r2n,'d_n2s',d_n2s,'d_s2e',d_s2e,'d_e2h',d_e2h,...
    'fig_pos',fig_pos2,'view_info',view_info,...
    'color','r','joi_sr',joi_sr,'axis_info',axis_info_ws);
plot_skeleton_from_features(x_bar,'fig_idx',2,'subfig_idx',2,...
    'd_r2n',d_r2n,'d_n2s',d_n2s,'d_s2e',d_s2e,'d_e2h',d_e2h,...
    'view_info','','color','k','joi_sr',joi_sr,'axis_info',axis_info_ws);

drawnow;


%%
