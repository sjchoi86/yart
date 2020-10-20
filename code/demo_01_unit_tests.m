ccc
%
% Here, we introduce some basic unit functions that will be used throughout this package
%
% 1. Make figures with different positions
% 2. Plot Homogeneous Transformation Matrices (Pre:global / Post:local)
% 3. Coordinate Transforms
% 4. Plot Cube in 3D and Rotate w.r.t. local-z
% 5. Plot 3D arrow and Rotate w.r.t. global-z
% 6. Kinematic Chain and Forward Kinematics
% 7. Get a Transformation Matrix from Three Points in 3D
% 8. Plot Interactive Markersc
% 9. Interpolate between Two Rotation Matrices, R1 and R2
% 10. Load URDF and Generate a Kinemactic Chain
% 11. Rodrigues' formula: get R from a constant angular velocity vector
% 12. Solve IK using a numerical method
% 13. Plot Kinematic Chain Structure
% 14. IK with Interactive Markers
% 15. Load CAD and find its bounding cube
% 16. Animate Chain with Linear and Angular Velocities
% 17. IK time profiling
% 18. Simple Dynamic Simulation (Rotation only)
% 19. Simple Dynamic Simulation (Translation and Rotation)
% 20. Convex Optimization with CVX (http://cvxr.com/cvx/download/)
% 21. Load CMU-MoCap DB and animate
% 22. Show graph of MoCap skeleton
% 23. Localty Sensitive Hashing (LSH)
% 24. Bayesian Optimization with Coordinate Descent
% 25. Select subset indices from an unordered set
% 26. Shortest path in an unweighted graph (BFS)
% 27. Voronoi optimistic optimization (VOO)
% 28. MoCap Skeleton Link Length Modification
% 29. Get the Robot Model with Bounding Capsules (and Caching)
% 30. Get Random Pose of a Robot Model and check self-collision
% 31. Get the Workspace of JOI of a robot
% 32. Rotate 3D vector with Slerp
% 33. Motion Retargeting Parametrization
% 34. Compute Linear/Angular Momentum
% 35. Compute ZMP
% 36. Slider Control
% 37. Get mu, d_mu, and dd_mu of GRP
% 38. GRP sampling
% 39. Surface plot
%
%% 1. Make figures with different positions
ccc
for tick = 1:2 % animate ticks
    set_fig_position(figure(1),'position',[0.0,0.6,0.2,0.35],'ADD_TOOLBAR',0,'view_info',[80,16],...
        'title_str',sprintf('Figure 1 tick:[%d]',tick));
    set_fig_position(figure(2),'position',[0.2,0.6,0.2,0.35],'ADD_TOOLBAR',0,'view_info',[80,16],...
        'title_str',sprintf('Figure 2 tick:[%d]',tick));
    set_fig_position(figure(3),'position',[0.4,0.6,0.2,0.35],'ADD_TOOLBAR',0,'view_info',[80,16],...
        'title_str',sprintf('Figure 3 tick:[%d]',tick));
    drawnow limitrate;
end

%% 2. Plot Homogeneous Transformation Matrices
% pre-multiply: global transfrom / post-multiply: local transform
ccc
% Random Transformation Matrix
T_init = pr2t(-0.5+1.0*rand(1,3),rpy2r(360*rand(1,3)));
% Plot figure
vid_obj = init_vid_record('../vid/unit_test/ut02_homogeneous_transforms.mp4','HZ',40,'SAVE_VID',0);
set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',2.0*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_T(T_init,'subfig_idx',2,'alw',1,'als','--','PLOT_AXIS_TIP',0); % plot initial T
plot_T(pr2t([0,0,0],eye(3,3)),'subfig_idx',3,'alen',1.0,'alw',3,'PLOT_AXIS_TIP',0,...
    'PLOT_SPHERE',0); % plot origin
for tick = 1:5:360 % Global rotate w.r.t. x-axis
    T = pr2t([0,0,0],rpy2r(tick*[1,0,0]*D2R))*T_init; % pre-mul
    plot_T(T,'alw',3); plot_title('Global X-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Global rotate w.r.t. y-axis
    T = pr2t([0,0,0],rpy2r(tick*[0,1,0]*D2R))*T_init; % pre-mul
    plot_T(T,'alw',3); plot_title('Global Y-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Global rotate w.r.t. z-axis
    T = pr2t([0,0,0],rpy2r(tick*[0,0,1]*D2R))*T_init; % pre-mul
    plot_T(T,'alw',3); plot_title('Global Z-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Local rotate w.r.t. x-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[1,0,0]*D2R)); % post-mul
    plot_T(T,'alw',3); plot_title('Local X-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Local rotate w.r.t. y-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[0,1,0]*D2R)); % post-mul
    plot_T(T,'alw',3); plot_title('Local Y-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Local rotate w.r.t. z-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[0,0,1]*D2R)); % post-mul
    plot_T(T,'alw',3); plot_title('Local Z-rotation'); drawnow; record_vid(vid_obj);
end
end_vid_record(vid_obj);

%% 3. Coordinate Transforms
ccc
T_world_coord = pr2t([0,0,0],eye(3,3)); % 'world coord'
% Randomly determine the initial 'local coord' and 'T_a_local'
T_w2a_coord_init = pr2t(1.0*rand(1,3),rpy2r(360*rand(1,3))); % initital 'local coord'
T_a_p = pr2t([0,0,-1],rpy2r(360*rand(1,3)*D2R)); % position in the 'a coord'

% Set figure
vid_obj = init_vid_record('../vid/unit_test/ut03_coordinate_transform.mp4','HZ',40,'SAVE_VID',0);
for tick = 1:1:360
    % 1. Change the 'local coord' by rotating w.r.t. the x-axis
    T_w2a_coord = T_w2a_coord_init*pr2t([0,0,0],rpy2r(tick*[1,0,0]*D2R));
    % Pre-multiply the 'local coord' to transfrom 'T_a_local' to the 'world coord'
    T_a = T_w2a_coord * T_a_p;
    % Animate
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',2.5*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(T_world_coord,'subfig_idx',1,'alen',1.0,'alw',4,'PLOT_AXIS_TIP',1,'atipsize',0.05,...
        'PLOT_SPHERE',1,'sr',0.05,'text_str','World Coord','text_fs',20); % 'world coord'
    plot_T(T_w2a_coord_init,'subfig_idx',2,'alen',0.5,'alw',2,'als','--','PLOT_AXIS_TIP',0,...
        'PLOT_SPHERE',1,'sr',0.1,'text_str',''); % 'local coord'
    plot_line(t2p(T_world_coord),t2p(T_w2a_coord),'subfig_idx',1,...
        'color','k','lw',2,'ls','--','text_str','w2l'); % line from 'world coord' to 'local coord'
    plot_T(T_w2a_coord,'subfig_idx',3,'alen',0.5,'alw',3,'PLOT_AXIS_TIP',1,'atipsize',0.1,...
        'PLOT_SPHERE',1,'sr',0.1,'text_str','Local Coord','text_fs',13); % 'local coord'
    plot_line(t2p(T_w2a_coord),t2p(T_a),'subfig_idx',2,...
        'color','b','lw',1,'ls','--','text_str','l2a'); % line from 'local coord' to 'p_a'
    plot_T(T_a,'subfig_idx',4,'alen',0.3,'alw',3,'atipsize',0.2,...
        'PLOT_SPHERE',1,'sr',0.1,'text_str','T_a'); % T_a in the 'world coord'
    plot_title(sprintf('[%d/%d]',tick,360));
    drawnow; record_vid(vid_obj);
    if ~ishandle(fig)
        break;
    end
end
end_vid_record(vid_obj);

%% 4. Plot Cube in 3D and Rotate w.r.t. local-z
ccc

T_world_coord = pr2t([0,0,0],eye(3,3)); % 'world coord'
T_cube_init = pr2t(2*rand(1,3),rpy2r(360*rand(1,3)*D2R)); % 'box coord'

% Set figure
vid_obj = init_vid_record('../vid/unit_test/ut04_plot_cube_3d.mp4','HZ',40,'SAVE_VID',0);
for tick = 1:360
    T_cube = T_cube_init*pr2t([0,0,0],rpy2r(tick*[0,0,1*D2R])); % local z-rotation
    
    % Animiate
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',5.0*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(T_world_coord,'subfig_idx',1,'alen',1.0,'alw',4,'PLOT_AXIS_TIP',1,'atipsize',0.05,...
        'PLOT_SPHERE',1,'sr',0.05,'text_str','World Coord','text_fs',20); % plot 'world coord'
    plot_T(T_cube_init,'subfig_idx',2,'alen',0.5,'alw',3,'als','--',...
        'PLOT_AXIS_TIP',1,'atipsize',0.15,...
        'PLOT_SPHERE',1,'sr',0.05,'text_str','Box Coord','text_fs',20); % plot 'box coord'
    plot_title('Rotate Cube w.r.t Local Z-axis');
    plot_line(t2p(T_world_coord),t2p(T_cube),'subfig_idx',1,...
        'color','k','lw',2,'ls','--','text_str','w2l'); % line from 'world coord' to 'box coord'
    plot_T(T_cube,'subfig_idx',3,'alen',0.5,'alw',3,'PLOT_AXIS_TIP',1,'atipsize',0.15,...
        'PLOT_SPHERE',1,'sr',0.05,'text_str','Box Coord','text_fs',20); % 'box coord'
    plot_cube(T_cube,[0,0,0],[1,2,3],'color','b','alpha',0.2,'edge_color','k'); % plot blue cube
    plot_title(sprintf('[%d/%d] Rotate w.r.t. Z-axis',tick,360));
    drawnow; record_vid(vid_obj);
    if ~ishandle(fig)
        break;
    end
end
end_vid_record(vid_obj);

%% 5. Plot 3D arrow and Rotate w.r.t. global-z
ccc

% Position of the tip of the arrow
p_tip_init = [0.5,1.0,1.5];
T_world_coord = pr2t([0,0,0],eye(3,3)); % 'world coord'

% Set figure
vid_obj = init_vid_record('../vid/unit_test/ut05_plot_arrow_3d.mp4','HZ',40,'SAVE_VID',0);
for tick = 1:360
    T_tip = pr2t([0,0,0],rpy2r(tick*[0,0,1]*D2R))*pr2t(p_tip_init,eye(3,3)); % rotate w.r.t. global z-axis
    p_tip = t2p(T_tip);
    
    % Animate
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',2.0*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(T_world_coord,'subfig_idx',1,'alen',1.0,'alw',4,'PLOT_AXIS_TIP',1,'atipsize',0.05,...
        'PLOT_SPHERE',1,'sr',0.05,'text_str','World Coord','text_fs',20); % plot 'world coord'
    plot_arrow_3d([0,0,0],p_tip,'color','b','alpha',0.4); % plot arrow
    plot_T(T_tip,'subfig_idx',2,'PLOT_AXIS',0,'PLOT_AXIS_TIP',0,...
        'PLOT_SPHERE',1,'sr',0.05,'text_str','Arrow Tip','text_fs',20);
    plot_title(sprintf('[%d/%d]',tick,360));
    drawnow; record_vid(vid_obj);
    if ~ishandle(fig)
        break;
    end
end
end_vid_record(vid_obj);

%% 6. Kinematic Chain and Forward Kinematics
%
% Here, we follow the parameters defined by Kajita et al. except we use multiple childs structure.
% We separate joints from links by adapting this multiple childs structure.
%
% Chain.joint contains:
%  name
%  parent
%  childs
%  p
%  R
%  v
%  w
%  q
%  dq
%  ddq
%  a        : joint axis vector (relative to parent)
%  b        : joint relative position (relative to parent)
%
% chain.link containts
%  m        : mass
%  c        : center of mass
%  I        : moment of intertia
%
ccc

% Manually define the kinematic chain
%
% Apply 'p_offset' first w.r.t parent's 'R', and then, apply 'R_offset'
%
chain.name = 'custom chain';
chain.joint(1) = struct('name','world','parent',[],'childs',[2],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[0,0,0]','p_offset',[1,0,0]','R_offset',rpy2r([0,0,0]));
chain.joint(2) = struct('name','j1','parent',[1],'childs',[3],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[0,0,1]','p_offset',[1,0,0]','R_offset',rpy2r([0,-90,0]));
chain.joint(3) = struct('name','j2','parent',[2],'childs',[4],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[0,1,0]','p_offset',[1,0,0]','R_offset',rpy2r([0,0,0]));
chain.joint(4) = struct('name','j3','parent',[3],'childs',[5],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[1,0,0]','p_offset',[1,0,0]','R_offset',rpy2r([0,0,0]));
chain.joint(5) = struct('name','ee','parent',[4],'childs',[],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[0,0,0]','p_offset',[1,0,0]','R_offset',rpy2r([0,0,0]));
chain.n_joint = length(chain.joint);
chain.n_revolute_joint = 2;
chain.joint_names = {'world','j1','j2','j3','ee'};
% Zero pose
chain = update_chain_q(chain,{'j1','j2','j3'},[0,0,0]);
chain = fk_chain(chain);
% Get size of the model
xyz_min = inf*ones(3,1);
xyz_max = -inf*ones(3,1);
for i_idx = 1:chain.n_joint
    xyz_min = min(xyz_min,chain.joint(i_idx).p);
    xyz_max = max(xyz_max,chain.joint(i_idx).p);
end
xyz_len = xyz_max - xyz_min;
chain.xyz_min = xyz_min;
chain.xyz_max = xyz_max;
chain.xyz_len = xyz_len;

vid_obj = init_vid_record('../vid/unit_test/ut06_kinematic_chain_fk.mp4','HZ',40,'SAVE_VID',0); % init video
max_tick = 360;
traj = nan*ones(max_tick,3);
for tick = 1:max_tick
    
    % Update position in radian
    qs = tick*[1,2,4]*D2R;
    chain = update_chain_q(chain,{'j1','j2','j3'},qs);
    
    % Forward kinematics
    chain = fk_chain(chain);
    
    % Plot Kinematic Chain
    fig = plot_chain(chain,'view_info',[88,4],'axis_info',[-2,+4,-3,+3,-4,+4],...
        'title_str',sprintf('[%d/%d]',tick,max_tick));
    % Plot End-effector
    T_ee = get_chain_T(chain,'ee');
    plot_cube(T_ee,[0.0,-0.2,-0.2],[1.0,0.4,0.4],'color','b','alpha',0.2,'edge_color','k');
    traj(tick,:) = t2p(T_ee)';
    plot_curve_3d(traj(:,1),traj(:,2),traj(:,3),'color',0.5*[1,1,1]);
    
    drawnow limitrate; record_vid(vid_obj);
    if ~ishandle(fig),break;end  % if figure is closed, then stop
end
end_vid_record(vid_obj);

%% 7. Get a Transformation Matrix from Three Points in 3D
ccc

% Three points in 3D
p1 = -1+2*rand(3,1); p2 = -1+2*rand(3,1); p3_init = -1+2*rand(3,1);

vid_obj = init_vid_record('../vid/unit_test/ut07_transform_from_three_points.mp4','HZ',40,'SAVE_VID',0); % init video

for tick = 1:360
    % Rotate p3
    p3 = rpy2r(tick*[0,0,1]'*D2R)*p3_init;
    % Get a transformation matrix from Three Points
    T = ps2T(p1,p2,p3);
    [p,R] = t2pr(T);
    % Plot
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',[-2,+2,-2,+2,-2,+2],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(p2t(p1),'subfig_idx',1,'PLOT_AXIS',0,'PLOT_AXIS_TIP',0,'sfc','r','sr',0.05,'sfa',0.9,...
        'text_str','p1'); % plot p1
    plot_T(p2t(p2),'subfig_idx',2,'PLOT_AXIS',0,'PLOT_AXIS_TIP',0,'sfc','g','sr',0.05,'sfa',0.9,...
        'text_str','p2'); % plot p2
    plot_T(p2t(p3),'subfig_idx',3,'PLOT_AXIS',0,'PLOT_AXIS_TIP',0,'sfc','b','sr',0.05,'sfa',0.9,...
        'text_str','p3'); % plot p3
    plot_T(T,'subfig_idx',4,'PLOT_AXIS',1,'alw',4,'PLOT_AXIS_TIP',0,'atipsize',0.05,...
        'sfc','k','sr',0.05,'text_str','T'); % plot T
    plot_plane([p1,p2,p3]','fc','y','ec','k','alpha',0.5);
    plot_arrow_3d(p,p+1.1*R(:,3),'color','b','alpha',0.5,'sw',0.1,'tw',0.2);
    plot_title(sprintf('[%d/%d] T from 3 points',tick,360));
    drawnow; record_vid(vid_obj);
    if ~ishandle(fig),break;end  % if figure is closed, then stop
end
end_vid_record(vid_obj);

%% 8. Plot Interactive Markers
ccc

% Plot interactive markers
vid_obj = init_vid_record('../vid/unit_test/ut08_interactive_markers.mp4','HZ',20,'SAVE_VID',0); % init video
fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',5*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_interactive_marker('subfig_idx',1,'T',pr2t([0,-2,0],rpy2r(0*rand(1,3))));
plot_interactive_marker('subfig_idx',2,'T',pr2t([0,+2,0],rpy2r(0*rand(1,3))));
tick = 0;
while ishandle(fig)
    tick = tick + 1;
    title_str = sprintf('[%d] p1:%s \n p2:%s \n',...
        tick,vec2str(t2p(g_im{1}.T)','%.2f'),vec2str(t2p(g_im{2}.T)','%.2f'));
    plot_title(title_str);
    drawnow limitrate; if ishandle(fig), record_vid(vid_obj); end
end
end_vid_record(vid_obj);

%% 9. Interpolate between Two Rotation Matrices, R1 and R2
ccc

% Set two rotation matrices
R1 = rpy2r(360*rand(1,3)*D2R);
R2 = rpy2r(360*rand(1,3)*D2R);
w = r2w(R1'*R2); % angular velocity in radian
w_hat = scew(w); % make sure to use radian here

vid_obj = init_vid_record('../vid/unit_test/ut09_interpolate_rotations.mp4','HZ',20,'SAVE_VID',0); % init video

for t = linspace(0,1,100)
    R_t = R1*expm(w_hat*t);
    % Plot
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',3*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(r2t(R1),'subfig_idx',1,'text_str','R1','TEXT_AT_ZTIP',1);
    plot_T(r2t(R2),'subfig_idx',2,'text_str','R2','TEXT_AT_ZTIP',1);
    plot_T(r2t(R_t),'subfig_idx',3,'alw',3,'PLOT_AXIS_TIP',1,...
        'text_str','R_t','TEXT_AT_ZTIP',1,'text_fs',20,'text_color','b');
    plot_title(sprintf('[%.2f] Interpolate R1 ad R2',t));
    drawnow; record_vid(vid_obj);
    
    if ~ishandle(fig), break; end
end
end_vid_record(vid_obj);

%% 10. Load URDF and Generate a Kinemactic Chain
ccc

% Which model to use
model_name = 'social_robot'; % coman / panda / sawyer / social_robot
urdf_path = sprintf('../urdf/%s/%s_urdf.xml',model_name,model_name);
chain = get_chain_from_urdf(model_name,urdf_path);
% Plot robot
chain = update_chain_q(chain,chain.rev_joint_names,360*ones(1,chain.n_rev_joint)*D2R);
chain = fk_chain(chain);
plot_chain(chain,'view_info',[88,4],'axis_info','',...
    'PLOT_MESH',1,'PLOT_LINK',1,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,...
    'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0,...
    'title_str',sprintf('[%s]',chain.name),...
    'MULTIPLE_MONITOR',0,'monitor_idx',1);
plot_T(pr2t([0,0,0]',rpy2r([0,0,0]')),'PLOT_SPHERE',0,'alw',4);
% Animate
ems_fk_total = 0;
max_tick = 100;
for tick = 1:max_tick
    chain = update_chain_q(chain,chain.rev_joint_names,tick*ones(1,chain.n_rev_joint)*D2R);
    i_clk = clock;
    chain = fk_chain(chain);
    ems_fk = etime(clock,i_clk)*1000;
    ems_fk_total = ems_fk_total + ems_fk;
    fig = plot_chain(chain,'view_info',[88,4],'axis_info','',...
        'PLOT_MESH',1,'PLOT_LINK',1,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,...
        'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0,...
        'title_str',sprintf('[%s][%d]',chain.name,tick));
    if ~ishandle(fig), break; end
    drawnow limitrate;
end
ems_fk_avg = ems_fk_total / max_tick;
fprintf('ems_fk_avg:[%.4f]ms.\n',ems_fk_avg);

%% 11. Rodrigues' formula: get R from a constant angular velocity vector
ccc
rng(0);
a_temp = randn(3,1);
a_init = a_temp / norm(a_temp); % unit vector
q_init = rand; % angular velocity
for tick = 1:5*360 % rotate axis
    a = rpy2r(tick*[0,0,1]'*D2R)*a_init;
    q = q_init;
    R = rodrigues(a,q); % rotation matrix from a and q
    % Plot
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',3*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(r2t(R),'subfig_idx',1,'text_str','R','TEXT_AT_ZTIP',1);
    plot_arrow_3d([0,0,0]',a*q);
    plot_title('Rodrigues Formula');
    drawnow;
    if ~ishandle(fig)
        break;
    end
end
for tick = 1:100 % increase the angular velocity
    a = a_init;
    q = q_init*(1+tick/50);
    R = rodrigues(a,q); % rotation matrix from a and q
    % Plot
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',3*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(r2t(R),'subfig_idx',1,'text_str','R','TEXT_AT_ZTIP',1);
    plot_arrow_3d([0,0,0]',a*q);
    plot_title('Rodrigues Formula');
    drawnow;
    if ~ishandle(fig)
        break;
    end
end

%% 12. Solve IK using a numerical method
ccc

% Get the kinematic chain of the model
model_name = 'coman'; % coman / panda / sawyer
urdf_path = sprintf('../urdf/%s/%s_urdf.xml',model_name,model_name);
chain = get_chain_from_urdf(model_name,urdf_path);
chain = update_chain_q(chain,chain.rev_joint_names,zeros(chain.n_rev_joint,1));
chain = fk_chain(chain); % initialize chain

% Initial IK
ik = init_ik(chain);
ik = add_ik(ik,'joint_name','RWrj2',...
    'p',chain.joint(idx_cell(chain.joint_names,'RWrj2')).p,'IK_P',1);
ik = add_ik(ik,'joint_name','LWrj2',...
    'p',chain.joint(idx_cell(chain.joint_names,'LWrj2')).p,'IK_P',1);

% Initialize the Kinematic Chain
q = (ik.joint_limits_lower+ik.joint_limits_upper)/2; % median pose
chain = update_chain_q(chain,ik.joint_names_control,q);
chain = fk_chain(chain); % initialize chain

% Start IK
while (ik.tick < ik.max_tick)
    
    % Run IK
    [ik,chain,q] = onestep_ik(ik,chain,q);
    
    % Check oscilation (limbo) of IK
    [FLAG,ik,best_err] = check_ik_oscilating(ik);
    if FLAG, chain = ik.chain; end
    
    % Plot IK status
    fig = plot_ik_chain(chain,ik);
    if FLAG || ~ishandle(fig) || (ik.err<1e-2)
        fprintf(2,'IK done.\n');
        break;
    end
end

%% 13. Plot Kinematic Chain Graph (structure)
ccc

% Get the kinematic chain of the model
model_name = 'social_robot'; % coman / panda / sawyer / social_robot
urdf_path = sprintf('../urdf/%s/%s_urdf.xml',model_name,model_name);
chain = get_chain_from_urdf(model_name,urdf_path);

% Plot chain graph
plot_chain_graph(chain);

%% 14. IK with Interactive Markers
ccc

% Get the kinematic chain of the model
model_name = 'panda'; % coman / panda / sawyer
urdf_path = sprintf('../urdf/%s/%s_urdf.xml',model_name,model_name);
chain = get_chain_from_urdf(model_name,urdf_path);

% Configuration
switch model_name
    case 'coman'
        ik_config(1) = struct('name','RWrj2','IK_P',1,'IK_R',0);
        ik_config(2) = struct('name','LWrj2','IK_P',1,'IK_R',0);
        ik_config(3) = struct('name','RElbj','IK_P',1,'IK_R',0);
        ik_config(4) = struct('name','LElbj','IK_P',1,'IK_R',0);
    case 'panda'
        ik_config(1) = struct('name','hand','IK_P',1,'IK_R',1);
    case 'sawyer'
        ik_config(1) = struct('name','right_hand','IK_P',1,'IK_R',1);
end

% Initialize IK with interactive marker
idxs = [];
for i_idx = 1:length(ik_config)
    idxs = union(idxs,get_idx_route(chain,ik_config(i_idx).name)); % all joints to targets
end
idxs = intersect(idxs,chain.rev_joint_idxs); % only for revolute joints
for i_idx = 1:length(ik_config)
    % idxs = setdiff(idxs,idx_cell(chain.joint_names,ik_config(i_idx).name)); % exclude target ones
end
joint_names_control = cell(1,length(idxs));
for i_idx = 1:length(idxs)
    joint_names_control{i_idx} = chain.joint(idxs(i_idx)).name;
end
ik = init_ik(chain,'joint_names_control',joint_names_control,...
    'stepsize',1.0*D2R,'stepsize_min',1.0*D2R,'stepsize_max',5.0*D2R,...
    'stepsize_inc_rate',2.0,'stepsize_dec_rate',0.5);
for i_idx = 1:length(ik_config)
    joint_name = ik_config(i_idx).name;
    ik = add_ik(ik,'joint_name',joint_name,...
        'p',chain.joint(idx_cell(chain.joint_names,joint_name)).p,...
        'R',chain.joint(idx_cell(chain.joint_names,joint_name)).R,...
        'IK_P',ik_config(i_idx).IK_P,'IK_R',ik_config(i_idx).IK_R);
end

% Save video
vid_obj = init_vid_record('../vid/unit_test/ut14_interactive_ik.mp4','HZ',20,'SAVE_VID',0); % init video

% Plot model and interactive markers
PLOT_LINK = 0; PLOT_JOINT_AXIS = 0; PLOT_JOINT_SPHERE = 0;
fig = plot_chain(chain,'PLOT_LINK',PLOT_LINK,'PLOT_JOINT_AXIS',PLOT_JOINT_AXIS,...
    'PLOT_JOINT_SPHERE',PLOT_JOINT_SPHERE);
for i_idx = 1:ik.n_target
    clen = max(chain.xyz_len)/10;
    plot_interactive_marker('subfig_idx',i_idx,'T',pr2t(ik.targets(i_idx).p,ik.targets(i_idx).R),...
        'clen',clen,'clen_er',1.2,'sr',clen/5);
end

% Run IK
q = get_q_chain(chain,ik.joint_names_control);
while ishandle(fig)
    % Run IK
    iclk_ik = clock;
    [ik,chain,q] = onestep_ik(ik,chain,q);
    ems_ik = etime(clock,iclk_ik)*1000;
    
    % Update IK target
    for i_idx = 1:ik.n_target
        T_i = g_im{i_idx}.T;
        [p_i,R_i] = t2pr(T_i);
        ik.targets(i_idx).p = p_i;
        ik.targets(i_idx).R = R_i;
    end
    
    % Animate
    title_str = sprintf('[%d] err:[%.3f] stepsize:[%.2f]deg [%.2f]ms',...
        ik.tick,ik.err,ik.stepsize*R2D,ems_ik);
    fig = plot_chain(chain,'title_str',title_str,'PLOT_LINK',PLOT_LINK,'PLOT_JOINT_AXIS',PLOT_JOINT_AXIS,...
        'PLOT_JOINT_SPHERE',PLOT_JOINT_SPHERE);
    drawnow limitrate; record_vid(vid_obj);
end
end_vid_record(vid_obj);
ca; % close all

%% 15. Load CAD and find its bounding cube
ccc

% Get the kinematic chain of the model
model_name = 'sawyer'; % coman / panda / sawyer
urdf_path = sprintf('../urdf/%s/%s_urdf.xml',model_name,model_name);
chain = get_chain_from_urdf(model_name,urdf_path);

for i_idx = 10
    link_i = chain.link(i_idx);
    
    if ~isempty(link_i.fv)
        % Get the bounding cube of the link
        bcube.xyz_min = min(link_i.fv.vertices)';
        bcube.xyz_max = max(link_i.fv.vertices)';
        bcube.xyz_len = bcube.xyz_max - bcube.xyz_min;
        bcube.c_offset = 0.5*(bcube.xyz_max + bcube.xyz_min);
        
        % Plot the CAD mesh and its bounding cube
        % T = pr2t([0,0,0],rpy2r(2*pi*rand(1,3))); % some random position
        T = pr2t(chain.joint(link_i.joint_idx).p,chain.joint(link_i.joint_idx).R); % actual link position
        set_fig_position(figure(1),'position',[0.5,0.6,0.2,0.35],'ADD_TOOLBAR',1,'view_info',[80,16]);
        h = patch('faces',link_i.fv.faces,'vertices',link_i.fv.vertices,...
            'FaceColor',0.5*[1,1,1],'EdgeColor','none','FaceLighting','gouraud',...
            'AmbientStrength',0.2,'FaceAlpha',0.3); % plot CAD
        tf = hgtransform;
        set(h,'parent',tf);
        set(tf,'Matrix',T); % move CAD to T
        plot_cube(T,bcube.xyz_min,bcube.xyz_len,'subfig_idx',i_idx,...
            'color','b','alpha',0.2,'edge_color','k'); % plot cube
        plot_T(T,'subfig_idx',2*i_idx,'alen',0.1,'alw',3,'PLOT_SPHERE',0); % basis axis
        T_com = T*p2t(bcube.c_offset); % CoM transformation in the global coordinates
        plot_T(T_com,'subfig_idx',1+2*i_idx+1,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.015,'sfc','r'); % link CoM
        plot_title(link_i.mesh_path);
    end
end

%% 16. Animate Chain with Linear and Angular Velocities
ccc

% Get the kinematic chain of the model
model_name = 'sawyer'; % coman / panda / sawyer
urdf_path = sprintf('../urdf/%s/%s_urdf.xml',model_name,model_name);
chain = get_chain_from_urdf(model_name,urdf_path);
chain.dt = 0.05; % set time step

% Animate
vid_obj = init_vid_record('../vid/unit_test/ut16_velocities.mp4','HZ',round(1/chain.dt),'SAVE_VID',0);
max_tick = 180;
vals = [linspace(0,+90,max_tick/2),linspace(90,0,max_tick/2)];
for tick = 1:max_tick
    chain = update_chain_q(chain,chain.rev_joint_names,vals(tick)*ones(chain.n_rev_joint,1)*D2R,...
        'IGNORE_LIMIT',0);
    chain = fk_chain(chain); % forward kinematics
    chain = fv_chain(chain); % forward velocities
    title_str = sprintf('[%.3f]sec Linear Velocity (red) Angular Velocity (blue)',tick*chain.dt);
    fig = plot_chain(chain,'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_BCUBE',0,'PLOT_COM',1,...
        'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,'PLOT_VELOCITY',1,'v_rate',0.5,'w_rate',0.3,...
        'title_str',title_str,'view_info',[35,47]);
    drawnow limitrate; record_vid(vid_obj);
    
    if ~ishandle(fig), break; end
end
end_vid_record(vid_obj);

%% 17. IK time profiling
%
% Set 'TIME_PROFILING' flag affects 'onestep_ik', 'compute_jacobian', and 'ik_ingredients'
%
% Calling 'idx_cell' is always a bottleneck. So we separated 'idx_cell' with 'idx_cell_multiple'.
%
ccc
global TIME_PROFILING
TIME_PROFILING = 1;

% Get the kinematic chain of the model
model_name = 'panda'; % coman / panda / sawyer
urdf_path = sprintf('../urdf/%s/%s_urdf.xml',model_name,model_name);
chain = get_chain_from_urdf(model_name,urdf_path);

% IK configuration
switch model_name
    case 'coman'
        ik_config(1) = struct('name','RWrj2','IK_P',1,'IK_R',0);
        ik_config(2) = struct('name','LWrj2','IK_P',1,'IK_R',0);
        ik_config(3) = struct('name','RElbj','IK_P',1,'IK_R',0);
        ik_config(4) = struct('name','LElbj','IK_P',1,'IK_R',0);
    case 'panda'
        ik_config(1) = struct('name','hand','IK_P',1,'IK_R',1);
    case 'sawyer'
        ik_config(1) = struct('name','right_hand','IK_P',1,'IK_R',1);
end
ik = get_ik_from_ik_config(chain,ik_config);

% Plot model and interactive markers
PLOT_LINK = 0; PLOT_JOINT_AXIS = 0; PLOT_JOINT_SPHERE = 0;
fig = plot_chain(chain,'PLOT_LINK',PLOT_LINK,'PLOT_JOINT_AXIS',PLOT_JOINT_AXIS,...
    'PLOT_JOINT_SPHERE',PLOT_JOINT_SPHERE);
for i_idx = 1:ik.n_target
    clen = max(chain.xyz_len)/10;
    plot_interactive_marker('subfig_idx',i_idx,'T',pr2t(ik.targets(i_idx).p,ik.targets(i_idx).R),...
        'clen',clen,'clen_er',1.2,'sr',clen/5);
end

% Run IK
q = get_q_chain(chain,ik.joint_names_control);
while ishandle(fig)
    
    % Run IK
    iclk_ik = clock;
    [ik,chain,q,~] = onestep_ik(ik,chain,q);
    ems_ik = etime(clock,iclk_ik)*1000;
    if TIME_PROFILING
        fprintf('IK took [%.2f]ms.\n',ems_ik);
    end
    
    % Update IK target with interactive markers
    for i_idx = 1:ik.n_target
        T_i = g_im{i_idx}.T;
        [p_i,R_i] = t2pr(T_i);
        ik.targets(i_idx).p = p_i;
        ik.targets(i_idx).R = R_i;
    end
    
    % Animate
    title_str = sprintf('[%d] err:[%.3f] stepsize:[%.2f]deg [%.2f]ms',...
        ik.tick,ik.err,ik.stepsize*R2D,ems_ik);
    fig = plot_chain(chain,'title_str',title_str,'PLOT_LINK',PLOT_LINK,'PLOT_JOINT_AXIS',PLOT_JOINT_AXIS,...
        'PLOT_JOINT_SPHERE',PLOT_JOINT_SPHERE);
    drawnow limitrate;
end

%% 18. Simple Dynamic Simulation (Rotation only)
ccc

% Make a cube
cube_size = 0.1*[1,2,3]'; % cube size
T_cube = pr2t([0,0,0]',rpy2r(360*rand(3,1))); % cube pose
density = 2710; % density of aluminum: 2,710kg/m3
mass = cube_size(1)*cube_size(2)*cube_size(3)*density;
I_cube_bar = diag([mass/12*(cube_size(2)^2+cube_size(3)^2),...
    mass/12*(cube_size(1)^2+cube_size(3)^2),...
    mass/12*(cube_size(1)^2+cube_size(2)^2)]); % moment of inertia [kgm2]
w = [30,30,30]'*D2R; % inital angular velocity [rad/s]
dt = 0.02; % integration time

max_tick = 500;
for tick = 1:max_tick
    % Update
    I_cube = t2r(T_cube)*I_cube_bar*t2r(T_cube)'; % update inertia
    L = I_cube*w; % angular momentum
    dw = I_cube \ (-cross(w,L)); % Euler's equation
    
    R = rodrigues(w/norm(w),norm(w)*dt) * t2r(T_cube); % Rodrigues
    R = rpy2r(r2rpy(R)); % make sure R is in SO(3)
    w = w + dt*dw; % Euler method
    T_cube = r2t(R); % update cube pose
    
    % Plot
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',0.5*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(T_cube,'subfig_idx',3,'alen',0.2,'alw',3,'PLOT_AXIS_TIP',1,'atipsize',0.15,...
        'PLOT_SPHERE',1,'sr',0.01,'text_str','Box Coord','text_fs',20); % box pos
    plot_cube(T_cube,-cube_size/2,cube_size,'color','b','alpha',0.2,'edge_color','k'); % plot cube
    p_cube = t2p(T_cube); % cube position
    plot_arrow_3d(p_cube,p_cube+w*0.3,'subfig_idx',1,'color','r'); % angular velocity (red)
    plot_arrow_3d(p_cube,p_cube+L*5.0,'subfig_idx',2,'color','y'); % angular momentum (yellow)
    plot_title(sprintf('[%d/%d][%.3f]sec',tick,max_tick,tick*dt));
    drawnow;
    if ~ishandle(fig), break; end
end

%% 19. Simple Dynamic Simulation (Translation and Rotation)
ccc

% Make a cube
cube_size = 0.1*[1,2,3]'; % cube size
T_cube = pr2t([0,0,0]',rpy2r(360*rand(3,1))); % cube pose
density = 2710; % density of aluminum: 2,710kg/m3
mass = cube_size(1)*cube_size(2)*cube_size(3)*density;
I_cube_bar = diag([mass/12*(cube_size(2)^2+cube_size(3)^2),...
    mass/12*(cube_size(1)^2+cube_size(3)^2),...
    mass/12*(cube_size(1)^2+cube_size(2)^2)]); % moment of inertia [kgm2]
w = [10,0,0]'*D2R; % inital angular velocity [rad/s]
vo = [0.3,0,0]'; % inital linear velocity [m/s]
dt = 0.02; % integration time

max_tick = 500;
for tick = 1:max_tick
    % Update
    [p,R] = t2pr(T_cube);
    norm_w = norm(w);
    if norm_w < eps
        p2 = p + dt*vo;
        R2 = R;
    else
        th = norm_w*dt;
        wn = w / norm_w;
        v = vo / norm_w;
        rot = rodrigues(wn, th);
        p2 = rot * p + (eye(3,3)-rot)*cross(wn,v) + wn*wn'*v*th;
        R2 = rot * R;
        R2 = rpy2r(r2rpy(R2)); % project to SO3
    end
    T_cube = pr2t(p2,R2);
    
    % Plot
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(T_cube,'subfig_idx',3,'alen',0.2,'alw',3,'PLOT_AXIS_TIP',1,'atipsize',0.15,...
        'PLOT_SPHERE',1,'sr',0.01,'text_str','Box Coord','text_fs',20); % box pos
    plot_cube(T_cube,-cube_size/2,cube_size,'color','b','alpha',0.2,'edge_color','k'); % plot cube
    p_cube = t2p(T_cube); % cube position
    plot_arrow_3d(p_cube,p_cube+w*0.3,'subfig_idx',1,'color','r'); % angular velocity (red)
    plot_title(sprintf('[%d/%d][%.3f]sec',tick,max_tick,tick*dt));
    drawnow;
    if ~ishandle(fig), break; end
end

%% 20. Convex Optimization with CVX (http://cvxr.com/cvx/download/)
%
% Run 'cvx_setup' in 'github/cvx'
% doc: http://cvxr.com/cvx/doc/CVX.pdf
%
ccc

% Least squares
m = 16; n = 8;
A = randn(m,n);
b = randn(m,1);
cvx_begin
variable x(n)
minimize( norm(A*x-b) )
cvx_end
fprintf('Our answer is:%s\n',vec2str(x'));

% Least squares with ineq constaints
m = 16; n = 8;
l = 0; u = 1; % set min and max
A = randn(m,n);
b = randn(m,1);
cvx_begin
variable x(n)
minimize( norm(A*x-b) )
subject to
l <= x <= u
cvx_end
fprintf('Our answer is:%s\n',vec2str(x'));

% Least squares with L1 norm
m = 16; n = 8;
A = randn(m,n);
b = randn(m,1);
cvx_begin
variable x(n)
minimize( norm(A*x-b,1) )
cvx_end
fprintf('Our answer is:%s\n',vec2str(x'));

% Least squares with other constaints (eq and norm)
m = 16; n = 8;
A = randn(m,n);
b = randn(m,1);
p = 4;
C = randn(p,n);
d = randn(p,1);
cvx_begin
variable x(n)
minimize( norm(A*x-b) )
subject to
C*x == d; % equality constraint should only be affine operations
norm(x,Inf) <= 1;
cvx_end
fprintf('Our answer is:%s\n',vec2str(x'));

% Quadratic programming with equality and inequality constraints
m = 16; n = 8;
A = randn(m,n);
b = randn(m,1);
Q = randn(m,m);
Q = Q'*Q;
p = 4;
C = randn(p,n);
d = randn(p,1);
l = [0*ones(n/2,1); -1*ones(n/2,1)];
u = [1*ones(n/2,1); 0*ones(n/2,1)];
cvx_begin
variable x(n)
minimize( quad_form( A*x - b, Q ) ) % == (A*x-b)'*Q*(Ax-b)
subject to
C*x == d;
l <= x <= u
cvx_end
fprintf('Our answer is:%s\n',vec2str(x'));

%% 21. Load CMU-MoCap DB and animate
%
% We assume that we cloned 'https://github.com/una-dinosauria/cmu-mocap'
% More info is 'https://sites.google.com/a/cgspeed.com/cgspeed/motion-capture/cmu-bvh-conversion'.
% at '../../cmu-mocap/
%
ccc

% Get all bvh paths of CMU MoCap DB
mocap_infos = get_bvh_infos('mocap_folder','../../cmu-mocap/');
n_mocap = length(mocap_infos);
fprintf('We have [%d] mocaps.\n',n_mocap);

% Get unique action strings
action_strings = cell(1,n_mocap);
for i_idx = 1:n_mocap % for all mocaps
    m = mocap_infos(i_idx);
    [mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/');
    if ~isnumeric(action_str)
        splits = strsplit(action_str,{'0',' ',','});
        action_strings{i_idx} = lower(splits{1});
    else
        action_strings{i_idx} = '';
    end
end
[~,unique_idx] = unique(action_strings);
unique_idx = sort(unique_idx); % sort
n_unique = length(unique_idx);
fprintf('We have [%d] unique mocaps.\n',n_unique);

% For all mocaps with unique action names
for i_idx = 1:n_unique % for mocaps with unique action names
    m = mocap_infos(unique_idx(i_idx));
    bvh_path = m.full_path;  % bvh path
    [mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/'); % CMU MoCap name
    fprintf('[%d/%d] [%s]-[%s] description:[%s].\n bvh_path:[%s].\n',...
        i_idx,n_unique,m.subject,m.action,action_str,bvh_path);
    
    ANIMATE = 1;
    if ANIMATE
        % Get skeleton and kinematic chains
        [skeleton,time] = load_raw_bvh(bvh_path);
        HZ = 30; % desired freq of mocap
        % Get a simple kinematic chain from skeleton @tick
        chains = get_chain_from_skeleton(skeleton,time,'USE_METER',1,'ROOT_AT_ORIGIN',1,'Z_UP',1,'HZ',HZ);
        L = min(length(chains),10*HZ); % length of chains (max if 10sec)
        
        % Animate skeleton
        ca; % close all
        vid_path = sprintf('../vid/cmumocapdb/%s.mp4',mocap_name);
        vid_obj = init_vid_record(vid_path,'HZ',HZ,'SAVE_VID',0); % save video?
        for tick = 1:L
            chain = chains{tick};
            title_str = sprintf('[%d/%d][%.2f]s [%s]',tick,L,tick/HZ,action_str);
            fig = plot_chain(chain,'fig_pos',[0.5,0.6,0.3,0.4],...
                'PLOT_LINK',1,'llw',2,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',1,'jal',0.08,...
                'PLOT_JOINT_SPHERE',1,'jsfc','c','jsr',0.03,'jsfa',0.7,'PRINT_JOINT_NAME',0,...
                'title_str',title_str,'tfs',13);
            plot_T(pr2t([0,0,0]',eye(3,3)),'PLOT_SPHERE',0);
            drawnow; record_vid(vid_obj);
            
            if ~ishandle(fig), break; end
        end
        end_vid_record(vid_obj); % save video
    end
    
    if ~ishandle(fig), break; end
end

%% 22. Show graph of MoCap skeleton
ccc

% Get all bvh paths of CMU MoCap DB
mocap_infos = get_bvh_infos('mocap_folder','../../cmu-mocap/');
m = mocap_infos(1);
bvh_path = m.full_path;  % bvh path
[skeleton,time] = load_raw_bvh(bvh_path);
chains = get_chain_from_skeleton(skeleton,time,'USE_METER',1,'ROOT_AT_ORIGIN',1,'Z_UP',1,'HZ',30);
[mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/'); % CMU MoCap name
chain = chains{1};

% Plot graph
plot_chain_graph(chain);

%% 23. Localty Sensitive Hashing (LSH)
ccc

N = 1e5; % # of data
D = 2; % # of dimensions of data
X = 10*rand(N,D); % [N x D]
L = 3; % # of tables
M = ceil(log(N)); % # of dimensions at projection space

nzr_x = init_nz(X);
nzd_x = get_nzdval(nzr_x,X);

% Make L hash tables
lshs = struct();
for l = 1:L % for each hash table
    A = randn(D, M); % random matrix
    w = 0; b = 0; % not used here
    
    Z = nzd_x*A; Z = Z ./ vecnorm(Z,2,2);
    Y_raw = 0.5*(sign( Z ) + 1); % binary
    min_Y = min(Y_raw);
    Y = Y_raw - min_Y; % min(Y(:))=0
    
    % Get unique codes
    [codes,~,bins] = unique(Y,'rows'); % codes:[B x M], bins:[N x 1]
    max_code = max(codes);
    n_code = size(codes,1);
    
    % Create Hash table
    table_sz = code2idx(max_code,max_code)+1;
    table = cell(1,table_sz);
    fprintf('Empty table [%d] ready.\n',table_sz);
    for n = 1:N % for each data index
        bin = bins(n);
        code = codes(bin,:); % vectorized bucket code of current data [1 x M]
        idx = code2idx(code,max_code)+1; % scalar bucket index of current data [1 x 1]
        table{idx} = [table{idx}, n]; % append the index of current data to the table
    end
    empties = cellfun(@isempty,table); % binary vector indicating empty buckets
    empty_rate = mean(empties);
    fprintf('[%d/%d] N:[%d] w:[%d] n_code:[%d] table_sz:[%d] empty_rate:[%.3f].\n',...
        l,L,N,w,n_code,table_sz,empty_rate);
    
    % (Heuristics) Fill-in empty buckets
    FILLIN_EMPTY_BUCKET = 0;
    if FILLIN_EMPTY_BUCKET
        for idx = find(empties==1) % for empty buckets
            code = idx2code(idx,max_code); % code of an empty bucket
            idx2 = code2idx(code,max_code)+1; % idx of an empty bucket
            % Find the closest nonempty bucket and append it
            [~,min_idx] = knn(code,codes,1);
            code_nonempty = codes(min_idx,:);
            idx_nonempty = code2idx(code_nonempty,max_code)+1;
            table{idx} = table{idx_nonempty};
        end
    end
    
    % Append
    lshs(l).A = A; lshs(l).b = b; lshs(l).w = w; lshs(l).min_Y = min_Y;
    lshs(l).codes = codes; lshs(l).bins = bins; lshs(l).max_code = max_code;
    lshs(l).table = table;
end

fprintf('Done.\n');

% Test
% Query with LSH
x_query = 10*rand(1,D); % query data
nzd_x_qeury = get_nzdval(nzr_x,x_query); % normalized x query

tic
bucket = [];
for l = 1:L % for each hash table
    A = lshs(l).A; b = lshs(l).b; w = lshs(l).w; min_Y = lshs(l).min_Y;
    max_code = lshs(l).max_code; table = lshs(l).table;
    
    Y_raw = 0.5*(sign( nzd_x_qeury*A ) + 1); % binary
    Y = Y_raw - min_Y; % min(Y(:))=0
    
    idx = code2idx(Y,max_code)+1;
    bucket = [bucket,table{idx}];
end
X_bucket = X(sort(unique(bucket)),:);
[~,min_idx] = knn(x_query,X_bucket,1); % knn
x_nn = X_bucket(min_idx,:);
toc

tic
[~,min_idx2] = knn(x_query,X,1);
x_nn2 = X(min_idx2,:);
toc

%% 24. Bayesian Optimization with Coordinate Descent
ccc

% Cost function
f = @(x)(f_simple_cost_in_2d(x));
max_val = 10;
g = set_grid(-max_val,max_val,100,-max_val,max_val,100);
g_Z = f(g.xy);

% Initialize BOCD
ranges(1).type = 'discrete';
ranges(1).range = linspace(-10,10,100);
ranges(2).type = 'continuous';
ranges(2).range = [-10,10];

bocd = init_bocd(ranges,f,'max_iter',100,'n_sample_for_acq',100,'n_burnin',1,'eps_prob',0.2,...
    'max_save',500,'n_bin',30,'max_bo_step',10,'max_cd_step',5,'VERBOSE',0);
mode_str = 'BOCD';

% Run BOCD
while ~bocd_finished(bocd)
    bocd = run_bocd(bocd);
    print_bocd(bocd);
    cost_added = bocd.cost_added;
    cost_best = bocd.cost_best;
    % Plot
    fig = figure(1); clf; hold on;
    set_fig_position(fig,'position',[0.1,0.5,0.3,0.4],'SET_DRAGZOOM',0);
    set(gcf,'color','w'); colormap copper;
    title(sprintf('[%d/%d] %s curr:[%.2f] best:[%.2f]',...
        bocd.iter,bocd.max_iter,mode_str,cost_added,cost_best),...
        'fontsize',20,'fontname','consolas');
    % mesh(g.xs,g.ys,reshape(g_Z,g.nx,g.ny),'FaceAlpha',0.5);
    pcolor(g.xs,g.ys,reshape(g_Z,g.nx,g.ny)); colorbar;
    plot(bocd.inputs(1:bocd.iter,1),bocd.inputs(1:bocd.iter,2),'x','color','b',...
        'MarkerSize',13,'LineWidth',2);
    plot(bocd.inputs(bocd.iter,1),bocd.inputs(bocd.iter,2),'p','color','w',...
        'MarkerSize',21,'LineWidth',3);
    axis equal; axis([-max_val,max_val,-max_val,max_val]); shading interp;
    drawnow;
end

% Summarize results
[best_input,best_cost] = get_bocd_result(bocd);
fprintf('best_cost:[%.3f] best_input:%s \n',best_cost,vec2str(best_input,'%.2f'));

%% 25. Select subset indices from an unordered set
ccc

x = [rand([100,2]); 0.2*rand([100,2])]; % imbalanced
ladpp_idx = get_sub_idx_from_unordered_set(x,10);
temp = randperm(size(x,1)); unif_idx = temp(1:10);
x_ladpp = x(ladpp_idx,:);
x_unif = x(unif_idx,:);
fig = figure();
set_fig_position(fig,'position',[0.1,0.5,0.4,0.5],'SET_DRAGZOOM',0);
hr = plot(x(:,1),x(:,2),'ko','MarkerSize',9,'LineWidth',2,'MarkerFaceColor','y');
hs = plot(x_ladpp(:,1),x_ladpp(:,2),'ro','MarkerSize',20,'LineWidth',3);
hu = plot(x_unif(:,1),x_unif(:,2),'bv','MarkerSize',20,'LineWidth',3);
grid on;
legend([hr,hs,hu],{'Raw','LA-DPP','Uniform'},'fontsize',25,'fontname','Consolas');

%% 26. Shortest path in an unweighted graph (BFS)
ccc

% Assume an Undirected Graph
n = 10000;
V = rand(n,2);
D = pdist2(V,V);
d_th = 0.03;
D(D>d_th) = 0; D(D<d_th & D~=0) = 1;
A = D;
v_src = randi([1,n]);
v_final = randi([1,n]);

% Precompute connections
B = cell(1,n);
for i_idx = 1:n
    B{i_idx} = find(A(i_idx,:)==1);
end

% Find
tic
[v_trace,n_check] = get_shortest_path_unweighted_graph(B,v_src,v_final);
sec = toc;
fprintf('BFS took [%.2f]ms and n_check is [%d].\n',1000*sec,n_check);

% Print
print_v_trace(v_trace);

% Plot
fig = figure();
set_fig_position(fig,'position',[0.1,0.5,0.3,0.45],'SET_DRAGZOOM',0);
hold on;
plot(V(:,1),V(:,2),'ko','markersize',3);
plot(V(v_src,1),V(v_src,2),'ro','markersize',15,'linewidth',3);
plot(V(v_final,1),V(v_final,2),'ro','markersize',15,'linewidth',3);
v_path = V(v_trace,:);
plot(v_path(:,1),v_path(:,2),'ro-','linewidth',3);
axis equal; axis off; axis([0,+1,0,+1]);
set(gcf,'color','w');
title(sprintf('Total [%d] points. BFS took [%.2f]ms and n_check is [%d]',n,sec*1000,n_check),...
    'fontsize',15,'fontname','consolas','interpreter','none');

%% 27. Voronoi optimistic optimization (VOO)
ccc

% Set grid (domain)
res = 200;
[xgrid,ygrid] = meshgrid(linspace(0,10,res),linspace(0,10,res));
xygrid = [xgrid(:),ygrid(:)];

% Set the cost function using GMM
mu = [8,2;2,8;8,8];
sigma = cat(3,[1.0,0.0;0.0,1.0],[1.0,0.0;0.0,1.0],...
    [1.0,0.0;0.0,1.0]);
probs = [1,1,2]; probs = probs / sum(probs);
gm = gmdistribution(mu,sigma,probs);
f = @(x)((gm.pdf(x))); % this will be our score function

% Run VOO
n_sample = 10000; dim = 2; max_exploit = 100; omega = 0.2;
px = @(n)(10*rand(n,2)); % sampler
voo = init_voo(n_sample,dim,max_exploit,omega,f,px);
tk = init_tk('VOO');
while voo_not_finished(voo)
    voo = one_step_voo(voo);
    tk = print_tk(tk,voo.tick,voo.n_sample);
end
voo = summarize_voo(voo);

% Plot
ca; fig = figure(1); set_fig_position(fig,'position',[0.0,0.5,0.35,0.5]); hold on;
fval = f(xygrid); fmtx = reshape(fval,res,res); % reshape
h = pcolor(xgrid,ygrid,fmtx); colormap(linspecer);
plot(voo.x_list(:,1),voo.x_list(:,2),'kx');
cb = colorbar; cb.FontSize = 15; cb.FontName = 'consolas';
set(h,'EdgeColor','none','FaceAlpha',0.3); set(gcf,'color','w');
axis([0,10,0,10]); axis('on', 'image'); axis off; dragzoom;

fig2 = figure(2); set_fig_position(fig2,'position',[0.35,0.5,0.35,0.5]); hold on;
ndhist(voo.x_list,'bins',1,'axis',[0,10,0,10]);
cb = colorbar; cb.FontSize = 15; cb.FontName = 'consolas';
set(h,'EdgeColor','none','FaceAlpha',0.3); set(gcf,'color','w');
axis('on', 'image'); axis off; dragzoom;

%% 28. MoCap Skeleton Link Length Modification
ccc

% Select one MoCap file
mocap_infos = get_bvh_infos('mocap_folder','../../cmu-mocap/');
n_mocap = length(mocap_infos);
fprintf('We have [%d] mocaps.\n',n_mocap);
m = mocap_infos(999); %
bvh_path = m.full_path;  % bvh path
[mocap_name,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/');
fprintf('[[%s]-[%s] description:[%s][%s].\n bvh_path:[%s].\n',...
    m.subject,m.action,mocap_name, action_str,bvh_path);
joi_mocap = get_joi_mocap(m.subject); % joints of interest of MoCap

% Get skeleton and kinematic chains
[skeleton,time] = load_raw_bvh(bvh_path);
HZ = 30;
chains_mocap = get_chain_from_skeleton(skeleton,time,...
    'USE_METER',1,'ROOT_AT_ORIGIN',1,'Z_UP',1,'HZ',HZ);
L = min(length(chains_mocap),5*HZ); % max 5sec
for tick = 1:L % for all tick
    chain_mocap = chains_mocap{tick};
    chain_mocap = upfront_chain(chain_mocap,joi_mocap); % upfront
    
    % Link Length Modificiation
    llm_vec = 1.0+rand(1,7);
    llm_stuct = struct('rh',llm_vec(1),'re',llm_vec(2),'rs',llm_vec(3),...
        'lh',llm_vec(4),'le',llm_vec(5),'ls',llm_vec(6),...
        'spine1',llm_vec(7),'neck',llm_vec(7));
    chain_mocap_llm = get_chain_mocap_llm(chain_mocap,joi_mocap,llm_stuct);
    
    % Plot the original MoCap skeleton
    fig1 = plot_chain(chain_mocap,...
        'fig_idx',1,'fig_pos',[0.0,0.6,0.3,0.4], ...
        'PLOT_LINK',1,'llw',2,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',1,'jal',0.02,...
        'PLOT_JOINT_SPHERE',1,'jsfc','c','jsr',0.02,'jsfa',0.1,'PRINT_JOINT_NAME',0,...
        'title_str',sprintf('[%d/%d][%.2f]s [%s]',tick,L,tick/HZ,action_str),'tfs',16);
    plot_joi_chain(chain_mocap,joi_mocap, ...
        'fig_idx',1,'sr',0.05,'sfa',0.5,'colors',linspecer(joi_mocap.n),...
        'PRINT_JOI_NAME',0);
    plot_T(pr2t([0,0,0]',eye(3,3)),'fig_idx',1,'PLOT_SPHERE',0);
    
    % Plot the link length modified MoCap skeleton
    fig2 = plot_chain(chain_mocap_llm,...
        'fig_idx',2,'fig_pos',[0.3,0.6,0.3,0.4], ...
        'PLOT_LINK',1,'llw',2,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',1,'jal',0.02,...
        'PLOT_JOINT_SPHERE',1,'jsfc','c','jsr',0.02,'jsfa',0.1,'PRINT_JOINT_NAME',0,...
        'title_str',sprintf('[%d/%d][%.2f]s [%s]',tick,L,tick/HZ,action_str),'tfs',16);
    plot_joi_chain(chain_mocap_llm,joi_mocap, ...
        'fig_idx',2,'sr',0.05,'sfa',0.5,'colors',linspecer(joi_mocap.n),...
        'PRINT_JOI_NAME',0);
    plot_T(pr2t([0,0,0]',eye(3,3)),'fig_idx',2,'PLOT_SPHERE',0);
    drawnow limitrate;
    
    if (~ishandle(fig1) || ~ishandle(fig2)), break; end
end

%% 29. Get the Robot Model with Bounding Capsules (and Caching)
ccc

% Load the robot model
model_name = 'coman'; % atlas / baxter / coman / panda / sawyer
chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');

% Plot
plot_chain(chain_model,...
    'fig_idx',1,'subfig_idx',1,'view_info',[88,4],'axis_info','',...
    'PLOT_MESH',1,'mfa',0.4,'PLOT_LINK',1,...
    'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',1,'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0,...
    'PLOT_CAPSULE',1,...
    'title_str',sprintf('[%s]',chain_model.name),...
    'MULTIPLE_MONITOR',0,'monitor_idx',1);

%% 30. Get Random Pose of a Robot Model and check self-collision
ccc

% Load the robot model
model_name = 'coman'; % atlas / baxter / coman / panda / sawyer
chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');
joi_model = get_joi_chain(chain_model);

% Plot chain graph
% plot_chain_graph(chain_model);

% Get joint names to control from JOI types
joi_types = {'rh','lh'};
jnames_ctrl = get_jnames_ctrl_from_joi_types(...
    chain_model,joi_model,joi_types,'EXCLUDE_TARGET',1);

for cnt = 1:10
    
    % Sample Random Robot Pose and Plot
    chain_model = update_chain_q(chain_model,jnames_ctrl,360*rand(1,length(jnames_ctrl))*D2R);
    chain_model = fk_chain(chain_model);
    [SC,sc_ij_list] = check_sc(chain_model); % check self-collision
    
    % Plot the Robot Model
    ral = chain_model.xyz_len(3)/10; rasw = ral/10; ratw = ral/5;
    fig = plot_chain(chain_model,...
        'fig_idx',1,'subfig_idx',1,'view_info',[88,4],'axis_info','',...
        'PLOT_MESH',1,'mfa',0.4,'PLOT_LINK',1,...
        'PLOT_ROTATE_AXIS',0,'ral',ral,'rasw',rasw,'ratw',ratw,...
        'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0,...
        'PLOT_CAPSULE',0,...
        'title_str',sprintf('[%s] SC:[%d]',chain_model.name,SC),...
        'MULTIPLE_MONITOR',0,'monitor_idx',1);
    clen = chain_model.xyz_len(3)/20;
    plot_joi_chain(chain_model,joi_model,'joi_types','','PLOT_COORD',1,'clen',clen);
    plot_sc_capsules(chain_model,sc_ij_list,'cfa',0.3); % plot self-collided capsules
    drawnow limitrate; pause;
    
    if ~ishandle(fig), break; end
end

%% 31. Get the Workspace of JOI of a robot
ccc

% Load the robot model
model_name = 'baxter'; % atlas / baxter / coman / panda / sawyer
ws_joi_types = {'rh','lh'}; % {'rh','re','lh','le'} / {'ee'}
chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');
joi_model = get_joi_chain(chain_model);

% Get the workspaces of the robot model
ws = get_ws_with_cache(chain_model,joi_model,'ws_joi_types',ws_joi_types,...
    'RE',0,'cache_folder','../cache','PLOT_EACH',1,'PLOT_FINAL',1);
for i_idx = 1:length(ws.joi_types)
    joi_type_i = ws.joi_types{i_idx};
    ws_info_i = ws.info{i_idx};
    fprintf('[%d][%s] min:%s max:%s len:%s.\n',i_idx,joi_type_i,...
        vec2str(ws_info_i.min_vals),vec2str(ws_info_i.max_vals),vec2str(ws_info_i.len_vals));
end

%% 32. Rotate 3D vector with Slerp
ccc

% Get the random vector
rvec = randn(3,1); rvec = 0.99*rvec/norm(rvec);
arrow_color = 0.5*[1,1,1];
res = 10; % resolution

% Plot
set_fig_position(figure(1),'position',[0.0,0.5,0.4,0.5],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_T(p2t(zeros(3,1)),'PLOT_SPHERE',0,'alw',3);
plot_arrow_3d(zeros(3,1),rvec,'subfig_idx',1,'color',arrow_color);
dir_vec = [1,0,0]'; dir_col = [1,0,0];
for r_idx = 1:res
    rotate_rate = r_idx/res;
    v_out = rotate_vec(rvec,dir_vec,rotate_rate);
    plot_arrow_3d(zeros(3,1),v_out,'subfig_idx',1+r_idx,...
        'color',rotate_rate*dir_col+(1-rotate_rate)*arrow_color);
end
dir_vec = [0,1,0]'; dir_col = [0,1,0];
for r_idx = 1:res
    rotate_rate = r_idx/res;
    v_out = rotate_vec(rvec,dir_vec,rotate_rate);
    plot_arrow_3d(zeros(3,1),v_out,'subfig_idx',1+res+r_idx,...
        'color',rotate_rate*dir_col+(1-rotate_rate)*arrow_color);
end
dir_vec = [0,0,1]'; dir_col = [0,0,1];
for r_idx = 1:res
    rotate_rate = r_idx/res;
    v_out = rotate_vec(rvec,dir_vec,rotate_rate);
    plot_arrow_3d(zeros(3,1),v_out,'subfig_idx',1+2*res+r_idx,...
        'color',rotate_rate*dir_col+(1-rotate_rate)*arrow_color);
end

%% 33. Motion Retargeting Parametrization
%
% Currently, it has 8 optimization variables
% - 4 for atease_rate
% - 4 for lengthen_rate
%
ccc

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
m = mocap_infos(randi([1,length(mocap_infos)])); % random mocap
bvh_path = m.full_path;  % bvh path
[~,action_str] = get_cmu_mocap_name(m,'mocap_folder','../../cmu-mocap/');
fprintf('[%s]-[%s] description:[%s].\n',m.subject,m.action,action_str);
[skeleton,time] = load_raw_bvh(bvh_path);
chains_mocap = get_chain_from_skeleton(skeleton,time,...
    'USE_METER',1,'ROOT_AT_ORIGIN',1,'Z_UP',1,'HZ',30);
L = length(chains_mocap);
joi_mocap = get_joi_mocap(m.subject);

% Loop
q_traj = zeros(L-1,n_ctrl);
for tick = 2:L
    
    % tick = 2; % tick
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
    q_traj(tick-1,:) = q'; % append
    chain_model = update_chain_q(chain_model,ik.joint_names_control,q);
    chain_model = fk_chain(chain_model); % initialize chain
    
    % Get JOI Positions of the Robot (pr: position of robot)
    [pr_root,pr_rs,pr_re,pr_rh,pr_ls,pr_le,pr_lh,pr_neck] = ...
        get_different_p_joi_robot_model(chain_model,joi_model);
    
    % Plot the robot model
    title_str = sprintf('[%d/%d] [%s]-[%s]',tick,L,model_name,action_str);
    axis_info = [-inf,inf,-1.1,1.1,chain_model_sz.xyz_min(3),1.1];
    fig = plot_chain(chain_model,'fig_idx',1,'subfig_idx',1,'view_info',[88,4],'axis_info',axis_info,...
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
    plot_spheres([pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr,pm_neck_mr]','subfig_idx',3,...
        'sr',joi_sr,'colors',joi_colors(2:end,:),'sfa',0.8);
    plot_lines([pm_root,pm_neck_mr,pm_rs_mr,pm_re_mr,pm_neck_mr,pm_ls_mr,pm_le_mr]',...
        [pm_neck_mr,pm_rs_mr,pm_re_mr,pm_rh_mr,pm_ls_mr,pm_le_mr,pm_lh_mr]',...
        'subfig_idx',1,'color','k','lw',3);
    drawnow;
    
    if ~ishandle(fig), break; end
end

%% 34. Compute Linear/Angular Momentum
ccc
% Load model
model_name = 'panda'; % atlas / baxter / coman / panda / sawyer
chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');
T = 0.01; chain_model.dt = T; % set time step
HZ = round(1/T); len = 1000;

momentum_scaler = 0.05;

for tick = 1:len % for each tick
    
    % Update model, dynamics and ZMP
    sec = tick * T; w = 5*pi/len;
    chain_model = update_chain_q(chain_model,{'joint1','joint2','joint3'},...
        [90*D2R*cos(w*tick),90*D2R*cos(w*tick),0*D2R*cos(w*tick)]);
    chain_model = fk_chain(chain_model); % forward kinematics
    chain_model = fv_chain(chain_model); % forward velocities
    q_rad = get_q_chain(chain_model,chain_model.joint_names)';
    if tick == 1, q_rad_diff = zeros(size(q_rad));
    else, q_rad_diff = q_rad - q_rad_prev;
    end
    q_rad_prev = q_rad;
    q_rad_diff = mod(q_rad_diff-pi,2*pi)+pi;
    
    % Update forward dynamic properties
    joints = chain_model.rev_joint_names;
    chain_model = update_chain_dynamics(chain_model,joints,q_rad_diff,T);
    
    title_str = sprintf('[%d/%d]',tick,len);
    axis_info = [-1.0,+1.0,-1.0,+1.0,+0.0,+2.0];
    fig = plot_chain(chain_model,'axis_info',axis_info,...
        'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_BCUBE',0,'PLOT_COM',1,...
        'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,'PLOT_VELOCITY',1,'v_rate',0.05,'w_rate',0.03,...
        'title_str',title_str,'view_info',[88,17]);
    sr = chain_model.xyz_len(3)/30;
    
    % Scale momentums for plotting
    P_arrow = chain_model.com' + [0, 0, 0; chain_model.P' * momentum_scaler];
    L_arrow = chain_model.com' + [0, 0, 0; chain_model.L' * momentum_scaler];
    [~,~,h_P] = plot_T(pr2t(P_arrow(2,:),eye(3,3)),'subfig_idx',1,...
        'PLOT_AXIS',0, 'PLOT_SPHERE',1,'sr',sr,'sfc','r','sfa',0.8);
    [~,~,h_L] = plot_T(pr2t(L_arrow(2,:),eye(3,3)),'subfig_idx',2,...
        'PLOT_AXIS',0, 'PLOT_SPHERE',1,'sr',sr,'sfc','b','sfa',0.8);
    
    if tick == 1
        h_P_arrow = plot3(P_arrow(:,1), P_arrow(:,2), P_arrow(:,3),'-', 'color','r','linewidth',3);
        h_L_arrow = plot3(L_arrow(:,1), L_arrow(:,2), L_arrow(:,3),'-', 'color','b','linewidth',3);
    else
        h_P_arrow.XData = P_arrow(:,1);
        h_P_arrow.YData = P_arrow(:,2);
        h_P_arrow.ZData = P_arrow(:,3);
        h_L_arrow.XData = L_arrow(:,1);
        h_L_arrow.YData = L_arrow(:,2);
        h_L_arrow.ZData = L_arrow(:,3);
    end
    if tick == 1
        h_leg = legend([h_P,h_L],{'$\mathcal{P}$','$\mathcal{L}$'}, ...
            'fontsize',25,'interpreter', 'latex',...
            'location','southeast','fontname','Consolas');
    end
    drawnow limitrate;
    if ~ishandle(fig), break; end
end

fprintf('Done.\n');

%% 35. Compute ZMP
ccc
% Load model
model_name = 'panda'; % atlas / baxter / coman / panda / sawyer
chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');
T = 0.01; chain_model.dt = T; % set time step
HZ = round(1/T); L = 1000;
w = 10*pi/L; % angular vel
com_traj = nan*ones(HZ,3); zmp_traj = nan*ones(HZ,3); % remain 1sec

for tick = 1:L % for each tick
    
    % Update model, dynamics and ZMP
    sec = tick * T;
    chain_model = update_chain_q(chain_model,{'joint1','joint2','joint3'},...
        [mod(90*D2R*(w*tick),2*pi),90*D2R + 0*D2R*cos(w*tick),0*D2R*cos(w*tick)],...
        'IGNORE_LIMIT',1);
    chain_model = fk_chain(chain_model); % forward kinematics
    chain_model = fv_chain(chain_model); % forward velocities
    q_rad = get_q_chain(chain_model,chain_model.joint_names)';
    if tick == 1, q_rad_diff = zeros(size(q_rad));
    else, q_rad_diff = q_rad - q_rad_prev;
    end
    q_rad_diff = mod(q_rad_diff+pi,2*pi)-pi;
    q_rad_prev = q_rad;
    
    % Update forward dynamic properties
    joints = chain_model.rev_joint_names;
    chain_model = update_chain_dynamics(chain_model,joints,q_rad_diff,T);
    if tick <= 2, dP = zeros(size(chain_model.P)); dL = zeros(size(chain_model.L));
    else % Compute numerical difference of P and L
        dP = (chain_model.P - model_P_prev)/T; dL = (chain_model.L - model_L_prev)/T;
    end
    model_P_prev = chain_model.P; model_L_prev = chain_model.L;
    zmp_z = chain_model.xyz_min(3);
    zmp = calc_model_zmp(chain_model,dP,dL,zmp_z); % compute ZMP
    com_proj = chain_model.com; com_proj(3) = zmp(3); % COM projected on the ground
    
    % Plot robot model and ZMP and COM projected on the ground
    com_traj(1:end-1) = com_traj(2:end); com_traj(end,:) = com_proj;
    zmp_traj(1:end-1) = zmp_traj(2:end); zmp_traj(end,:) = zmp;
    title_str = sprintf('[%d/%d] time:[%.2f]s',tick,L,sec);
    axis_info = [-1.0,+1.0,-1.0,+1.0,+0.0,+2.0];
    fig = plot_chain(chain_model,'axis_info',axis_info,...
        'PLOT_LINK',0,'PLOT_ROTATE_AXIS',0,'PLOT_BCUBE',0,'PLOT_COM',1,'PLOT_CAPSULE',0,...
        'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,'PLOT_VELOCITY',1,'v_rate',0.05,'w_rate',0.03,...
        'title_str',title_str,'view_info',[88,17]);
    sr = chain_model.xyz_len(3)/30;
    [~,~,h_zmp] = plot_T(pr2t(zmp,eye(3,3)),'subfig_idx',1,'PLOT_AXIS',0,...
        'PLOT_SPHERE',1,'sr',sr,'sfc','r','sfa',0.8);
    [~,~,h_com] = plot_T(pr2t(com_proj,eye(3,3)),'subfig_idx',2,'PLOT_AXIS',0,...
        'PLOT_SPHERE',1,'sr',sr,'sfc','b','sfa',0.8);
    if tick == 1
        h_zmp_traj = plot3(zmp_traj(:,1),zmp_traj(:,2),zmp_traj(:,3),'-',...
            'color','r','linewidth',3);
        h_com_traj = plot3(com_traj(:,1),com_traj(:,2),com_traj(:,3),'-',...
            'color','b','linewidth',3);
    else
        h_zmp_traj.XData = zmp_traj(:,1);
        h_zmp_traj.YData = zmp_traj(:,2);
        h_zmp_traj.ZData = zmp_traj(:,3);
        h_com_traj.XData = com_traj(:,1);
        h_com_traj.YData = com_traj(:,2);
        h_com_traj.ZData = com_traj(:,3);
    end
    if tick == 1
        h_leg = legend([h_zmp,h_com],{'ZMP','COM'},...
            'fontsize',25,'location','southeast','fontname','Consolas');
    end
    drawnow limitrate;
    if ~ishandle(fig), break; end
end

fprintf('Done.\n');

%% 36. Slider control
ccc

model_name = 'panda'; % atlas / baxter / coman / panda / sawyer
chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');

% Trajectory
HZ = 30;
L = 200;
degs_traj = zeros(L,7);
for tick = 1:L
    degs_traj(tick,:) = 180*D2R*sin(1*pi*tick/L)*[1,1,1,1,1,1,1];
end

% Precompute modelsmn
chain_models = cell(1,L);
for tick = 1:L
    chain_model = update_chain_q(chain_model,chain_model.rev_joint_names,degs_traj(tick,:));
    chain_model = fk_chain(chain_model);
    chain_models{tick} = chain_model;
end

% Figure
fig = figure(1);
view_info = [88,16];
set_fig_position(fig,'position',[0.0,0.4,0.4,0.5],'view_info',view_info); % the set size

% Add slider and button
slider_max = L; % max slider tick
sb = add_slider_and_button_to_figure(fig,slider_max,HZ,'y_offset',0.005,'y_height',0.03,...
    'sliderstep',1/40,'fontsize',12,'VERBOSE',1);

while ishandle(fig)
    
    % Process slider and button
    sb = process_slider_and_button(sb);
    
    % Update model
    chain_model = chain_models{sb.tick_slider};
    
    % Plot model
    title_str = sprintf('wall:[%.2f]s [%d/%d] sim:[%.2f]s',...
        sb.sec_wall,sb.tick_slider,sb.slider_max,sb.sec_sim);
    plot_chain(chain_model,'fig_pos','','title_str',title_str);
    drawnow;
end
fprintf('Done.\n');

%% 37. Basics of GRP mu, d_mu, and dd_mu
ccc

% Reference data
f_ref = @(x)( cos(x) ); % reference function
t_max = 4*pi;
t_ref = linspace(0,t_max,1000)';
x_ref = f_ref(t_ref);

% Anchor dataset (training data)
n_anchor = 20;
t_anchor = linspace(0,t_max,n_anchor)';
noise_var = 0e-2;
x_anchor = f_ref(t_anchor) + sqrt(noise_var)*randn(size(t_anchor));
l_anchor = ones(n_anchor,1);

% Gaussian random path
n_test = 1000;
t_test = linspace(0,t_max,n_test)';
hyp = [1,2]; % [gain,len]
[k_test,dk_test,ddk_test] = kernel_levse(t_test,t_anchor,ones(n_test,1),l_anchor,hyp);
K_anchor = kernel_levse(t_anchor,t_anchor,l_anchor,l_anchor,hyp);

% Compute mu, d_mu, and dd_mu
meas_std = 1e-1; % expected noise
mu_test = k_test / (K_anchor+meas_std*eye(n_anchor,n_anchor)) * x_anchor;
dmu_test = dk_test / (K_anchor+meas_std*eye(n_anchor,n_anchor)) * x_anchor;
ddmu_test = ddk_test / (K_anchor+meas_std*eye(n_anchor,n_anchor)) * x_anchor;

% Plot
fig = figure();
set_fig_position(fig,'position',[0.0,0.4,0.5,0.4],'AXIS_EQUAL',0,'SET_DRAGZOOM',0);
h_anchor = plot(t_anchor,x_anchor,'bo','linewidth',2,'markersize',15);
h_ref = plot(t_ref,x_ref,'k-','linewidth',5);
h_mu = plot(t_test,mu_test,'r--','linewidth',4);
h_dmu = plot(t_test,dmu_test,'-','linewidth',2,'Color','m');
h_ddmu = plot(t_test,ddmu_test,'-','linewidth',2,'Color','c');
legend([h_ref,h_anchor,h_mu,h_dmu,h_ddmu],{'Ref','Anchor','mu','d_mu','dd_mu'},'fontsize',15,...
    'interpreter','none');
ylim([-1,+1]*1.5);

%% 38. GRP sampling
ccc

% Anchor points
eps_ru = 0.01;
t_anchor = [0,1/3,2/3,1.0]';
x_anchor = [0,1.0,-1.0,3.0]';
l_anchor = [1,1-0.1,1-0.5,1]';

% Test data
n_test = 1000;
t_test = linspace(0,1,n_test)';

% Hyper parameters
hyp_mu = [1,1.0]; % [gain,len]
hyp_var = [1,0.1]; % [gain,len]


% Add epsilon run-up
[t_anchor,x_anchor,l_anchor] = add_eps_ru(t_anchor,x_anchor,l_anchor,eps_ru);
n_anchor = size(t_anchor,1);

% Compute GP mu and std
k_test_mu = kernel_levse(t_test,t_anchor,ones(n_test,1),ones(n_anchor,1),hyp_mu);
K_anchor_mu = kernel_levse(t_anchor,t_anchor,ones(n_anchor,1),ones(n_anchor,1),hyp_mu);
eps_mu = 1e-8; % expected noise
mu_test = k_test_mu / (K_anchor_mu+eps_mu*eye(n_anchor,n_anchor)) * x_anchor;

% Sample trajectories from GRP
k_test_var = kernel_levse(t_test,t_anchor,ones(n_test,1),l_anchor,hyp_var);
K_anchor_var = kernel_levse(t_anchor,t_anchor,l_anchor,l_anchor,hyp_var);
eps_var = 1e-8;
var_test = diag(abs(hyp_var(1) - ...
    k_test_var / (K_anchor_var+eps_var*eye(n_anchor,n_anchor)) * k_test_var'));
std_test = sqrt(var_test);
K_test = kernel_levse(t_test,t_test,ones(n_test,1),ones(n_test,1),hyp_var) - ...
    k_test_var / (K_anchor_var+eps_var*eye(n_anchor,n_anchor)) * k_test_var';
K_test = 0.5*K_test + 0.5*K_test';
K_max = max(K_test(:));
chol_K = chol(K_test/sqrt(K_max) + 1e-6*eye(n_test,n_test),'lower');

% Sample GRP paths
n_path = 100;
randomness = randn(n_test,n_path);
sampled_trajs = sqrt(K_max)*chol_K*randomness + mu_test;

% Plot
fig = figure();
set_fig_position(fig,'position',[0.0,0.4,0.5,0.4],'AXIS_EQUAL',0,'SET_DRAGZOOM',0,'AXES_LABEL',0);
h_fill = fill([t_test;flipud(t_test)],[mu_test-2*std_test ;flipud(mu_test+2*std_test)],...
    'k','LineStyle',':'); % grp 2std
set(h_fill,'FaceAlpha',0.2);
colors = linspecer(n_path);
for i_idx = 1:n_path
    sampled_traj = sampled_trajs(:,i_idx);
    color = colors(i_idx,:); % 0.4*[1,1,1];
    plot(t_test,sampled_traj,'-','linewidth',1.0,'color',color);
end
h_anchor = plot(t_anchor,x_anchor,'ko','linewidth',3,'markersize',15);
h_mu = plot(t_test,mu_test,'k-','linewidth',4);
legend([h_anchor,h_mu,h_fill],{'Data','GP mu','GP std'},'fontsize',20,'location','southeast');

%% 39. Surface plot
ccc

[rho,w] = meshgrid(-4:0.1:1,-5:0.2:5);
[rho_line,w_line] = meshgrid(-4:0.1:1,1);
alpha = 1;
zeta = 0.5;

s = rho + 1i*w;
G = (s + alpha)./(s.*s + 2*zeta*s + 1);
norm_G = abs(G);
angle_G = angle(G);

s_line = rho_line + 1i*w_line;
G_line = (s_line + alpha)./(s_line.*s_line + 2*zeta*s_line + 1);
norm_G_line = abs(G_line);
angle_G_line = angle(G_line);

figure(1); hold on;
colormap summer;
surf(rho,w,norm_G,'linewidth',0.5,'FaceAlpha',0.5,'edgecolor','interp'); % 'edgecolor':'none'/'interp'
plot3(rho_line,w_line,norm_G_line,'k-','linewidth',3);
xlabel('\rho'); ylabel('w');
view(155,26);

%% 40. GRP wrapper (1D case)
ccc

% Anchor points
t_anchor = [0,1/3,2/3,1.0]';
x_anchor = [0,1.0,-1.0,3.0]';
l_anchor = [1,1-0.1,1-0.5,1]';
eps_ru = 0.01;

% Test data
n_test = 1000;
t_test = linspace(0,1,n_test)';

% Hyper parameters
hyp_mu = [1,1.0]; % [gain,len]
hyp_var = [1,0.2]; % [gain,len]

% Initialize GRP
grp1d = init_grp1d(t_anchor,x_anchor,l_anchor,t_test,hyp_mu,hyp_var,'eps_ru',eps_ru);

% Sample GRP paths
n_path = 1;
randomness = randn(n_test,n_path);
sampled_trajs = sqrt(grp1d.K_max)*grp1d.chol_K*randomness + grp1d.mu_test;

% Plot
plot_grp1d(grp1d,sampled_trajs);

%% 41. GRP vel estimator wrapper (1D case)
ccc

% Anchor points
t_anchor = [0.0,2.0]'; % 2 second trajectory
x_anchor = [2*rand-1,2*rand-1]';
l_anchor = [1,1]';
eps_ru = 0.02;

% Test data
n_test = 1000;
t_test = linspace(0.0,2.0,n_test)';

% Hyper parameters
hyp_mu = [1,1.0]; % [gain,len]
hyp_var = [0.5,0.5]; % [gain,len]

% Initialize GRP
grp1d = init_grp1d(t_anchor,x_anchor,l_anchor,t_test,hyp_mu,hyp_var,'eps_ru',eps_ru);

% Sample GRP paths
n_path = 10;
randomness = randn(n_test,n_path);
sampled_trajs = sqrt(grp1d.K_max)*grp1d.chol_K*randomness + grp1d.mu_test;

% Compute the velocity of each sampled path
for p_idx = 1:n_path
    sampled_traj = sampled_trajs(:,p_idx); % get each path
    [min_val,max_val,min_vel,max_vel,aa_vel] = ...
        get_path_stats(t_test,sampled_traj);
    
    fprintf('[%02d/%d] val:[%.2f]~[%.2f] vel:[%.2f]~[%.2f] (aa:[%.2f]). \n',...
        p_idx,n_path,min_val,max_val,min_vel,max_vel,aa_vel);
end

% Plot
axis_info = [0,2,-2,+2];
plot_grp1d(grp1d,sampled_trajs,'fig_idx',1,'lw_sample',2,'axis_info',axis_info);

%% 42. Squash trajs
ccc

raw_traj = linspace(-10,10,1000)';

% Plot with different margins
fig = figure(1); set_fig_position(fig,'position',[0,0.5,0.3,0.5]);
plot(raw_traj,raw_traj,'k-','linewidth',2);
margins = [0.0,1.0,2.0,3.0,4.0,5.0];
n_margin = length(margins);
colors = linspecer(n_margin);
min_vals = -4;
max_vals = 2;
for m_idx = 1:n_margin
    margin = margins(m_idx);
    squashed_traj = get_squashed_traj(raw_traj,min_vals,max_vals,margin);
    hs(m_idx) = plot(raw_traj,squashed_traj,'-','linewidth',2,'color',colors(m_idx,:));
    strs{m_idx} = sprintf('margin:[%.1f]',margin);
end
grid on; axis equal; axis ([-10,+10,-10,+10]);
legend(hs,strs,'fontsize',15,'location','southeast');

% Plot with different limits
fig = figure(2); set_fig_position(fig,'position',[0.3,0.5,0.3,0.5]);
plot(raw_traj,raw_traj,'k-','linewidth',2);
limits = [1,2,3,4,5,6,7,8];
n_limit = length(limits);
colors = linspecer(n_limit);
for m_idx = 1:n_limit
    limit = limits(m_idx);
    squashed_traj = get_squashed_traj(raw_traj,-limit,limit,1);
    hs(m_idx) = plot(raw_traj,squashed_traj,'-','linewidth',2,'color',colors(m_idx,:));
    strs{m_idx} = sprintf('limit:[%.1f]',limit);
end
grid on; axis equal; axis ([-10,+10,-10,+10]);
legend(hs,strs,'fontsize',15,'location','southeast');

%% 43. GRP sampling of Panda manipulator
ccc

SAVE_VID = 0;

% Load model
model_name = 'panda'; % atlas / baxter / coman / panda / sawyer
chain_model = get_chain_model_with_cache(model_name,...
    'RE',0,'cache_folder','../cache','urdf_folder','../urdf');
joi_model = get_joi_chain(chain_model);
ws = get_ws_with_cache(chain_model,joi_model);
chain_model_sz = get_chain_sz(chain_model);
axis_info = get_axis_info_from_chain(ws,chain_model_sz);

% Tracklet configuration
t_sec = 2;
HZ = 50;
joint_vel_deg_max = 60; % deg/sec

% Sample joint tracket with GRP
joint2use = chain_model.rev_joint_names; % joints to use
joint2use = {'joint1','joint2'};
[q_tracklet,chain_model1,chain_model2] = sample_q_tracklet_with_grp(...
    chain_model,joi_model,joint2use,t_sec,HZ,joint_vel_deg_max);

% Animate the robot
if SAVE_VID
    vid_obj = init_vid_record('../vid/unit_test/ut43_grp_tracklet_panda.mp4','HZ',HZ,'SAVE_VID',1);
end
title_str = '';
plot_chain(chain_model1,'fig_idx',1,'subfig_idx',1,...
    'fig_pos',[0.0,0.1,0.6,0.9],'title_str',title_str,...
    'PLOT_LINK',0,'PLOT_CAPSULE',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,...
    'mfc',0.9*[1,1,1],'mfa',0.1,'axis_info',axis_info);
plot_chain(chain_model2,'fig_idx',1,'subfig_idx',2,...
    'fig_pos',[0.0,0.1,0.6,0.9],'title_str',title_str,...
    'PLOT_LINK',0,'PLOT_CAPSULE',0,'PLOT_ROTATE_AXIS',0,'PLOT_JOINT_AXIS',0,'PLOT_JOINT_SPHERE',0,...
    'mfc',0.9*[1,1,1],'mfa',0.1,'axis_info',axis_info);
for tick = 1:t_sec*HZ % for each tick
    q = q_tracklet(tick,:);
    chain_model = update_chain_q(chain_model,joint2use,...
        q,'IGNORE_LIMIT',0);
    chain_model = fk_chain(chain_model); % forward kinematics
    title_str = sprintf('[%d/%d] [%.2f]sec',tick,t_sec*HZ,tick/HZ);
    plot_chain(chain_model,'fig_idx',1,'subfig_idx',3,...
        'fig_pos',[0.0,0.1,0.6,0.9],'title_str',title_str,...
        'PLOT_LINK',0,'PLOT_CAPSULE',0,'axis_info',axis_info);
    drawnow;
    if SAVE_VID
        record_vid(vid_obj);
    end
end
drawnow;
if SAVE_VID
    end_vid_record(vid_obj);
end

%% 44. s(t) function parametrization with Normalized Wahba-type spline model
ccc

n_sample = 20;
sts = cell(1,n_sample);
for s_idx = 1:n_sample
    n_anchor = 7; range = 2; % Anchor points
    x_anchor = -range+2*range*rand(n_anchor,1); x_anchor(1)=0.0; x_anchor(end)=0.0;
    [st,t_test] = get_st_graph(x_anchor); % Get s(t)
    sts{s_idx} = st; % append
end

% Plot
fig = figure(1);
set_fig_position(fig,'position',[0.0,0.5,0.3,0.5],'AXIS_EQUAL',0,'SET_DRAGZOOM',0,'AXES_LABEL',0);
colors = linspecer(n_sample);
for s_idx = 1:n_sample
    st = sts{s_idx};
    color = colors(s_idx,:);
    plot(t_test,st,'-','linewidth',1,'Color',color);
end
axis equal;

%% 45. Invsere mapping from s(t) to w(t)
ccc

% Sample s(t)
n_anchor = 7; range = 2; % Anchor points
x_anchor = -range+2*range*rand(n_anchor,1); x_anchor(1)=0.0; x_anchor(end)=0.0;
[st,t_test] = get_st_graph(x_anchor); % Get s(t)

% Find corresponding w(t)
w = get_w_from_st(st);

% Find the anchor (representation)
n_anchor2 = 10;
idxs = round(linspace(1,length(t_test),n_anchor2));
x_anchor2 = w(idxs);

% Reconstruct s(t)
[st2,t_test] = get_st_graph(x_anchor2); % Get s(t)

% Plot s(t)
set_fig_position(figure(1),'position',[0.0,0.5,0.3,0.5],...
    'AXIS_EQUAL',0,'SET_DRAGZOOM',0,'AXES_LABEL',0);
hs = plot(t_test,st,'-','linewidth',2,'Color','g');
hsr = plot(t_test,st2,':','linewidth',2,'Color','r');
legend([hs,hsr],{'s(t)','Reconstructed s(t)'},...
    'fontsize',15,'location','southeast');
axis equal; axis([0,1,0,1]);

%%



























