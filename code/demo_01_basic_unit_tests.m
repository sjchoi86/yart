ccc
%
% Here, we introduce some basic unit functions that will be used throughout this package
%
% 1. Make figures with different positions
% 2. Plot Homogeneous Transformation Matrices (Pre:global / Post:local)
% 3. Coordinate Transforms
% 4. Plot Cube in 3D
% 5. Plot 3D arrow
% 6. Kinematic Chain and Forward Kinematics
% 7. Get a Transformation Matrix from Three Points in 3D
% 8. Plot Interactive Markers
% 9. Interpolate between Two Rotation Matrices, R1 and R2
%
%% 1. Make figures with different positions
ccc

set_fig_position(figure(1),'position',[0.0,0.6,0.2,0.35],'ADD_TOOLBAR',1,'view_info',[80,16]);
set_fig_position(figure(2),'position',[0.2,0.6,0.2,0.35],'ADD_TOOLBAR',1,'view_info',[80,16]);
set_fig_position(figure(3),'position',[0.4,0.6,0.2,0.35],'ADD_TOOLBAR',1,'view_info',[80,16]);

%% 2. Plot Homogeneous Transformation Matrices
% pre-multiply: global transfrom / post-multiply: local transform
ccc
% Random Transformation Matrix
T_init = pr2t(-0.5+1.0*rand(1,3),rpy2r(360*rand(1,3)));
% Plot figure
vid_obj = init_vid_record('../vid/ut02_homogeneous_transforms.mp4','HZ',40,'SAVE_VID',1);
set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',2.0*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_T(T_init,'subfig_idx',2,'alw',1,'als','--','PLOT_AXIS_TIP',0); % plot initial T
plot_T(pr2t([0,0,0],eye(3,3)),'subfig_idx',3,'alen',1.0,'alw',3,'PLOT_AXIS_TIP',0,...
    'PLOT_SPHERE',0); % plot origin
for tick = 1:5:360 % Global rotate w.r.t. x-axis
    T = pr2t([0,0,0],rpy2r(tick*[1,0,0]))*T_init; % pre-mul
    plot_T(T,'alw',3); plot_title('Global X-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Global rotate w.r.t. y-axis
    T = pr2t([0,0,0],rpy2r(tick*[0,1,0]))*T_init; % pre-mul
    plot_T(T,'alw',3); plot_title('Global Y-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Global rotate w.r.t. z-axis
    T = pr2t([0,0,0],rpy2r(tick*[0,0,1]))*T_init; % pre-mul
    plot_T(T,'alw',3); plot_title('Global Z-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Local rotate w.r.t. x-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[1,0,0])); % post-mul
    plot_T(T,'alw',3); plot_title('Local X-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Local rotate w.r.t. y-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[0,1,0])); % post-mul
    plot_T(T,'alw',3); plot_title('Local Y-rotation'); drawnow; record_vid(vid_obj);
end
for tick = 1:5:360 % Local rotate w.r.t. z-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[0,0,1])); % post-mul
    plot_T(T,'alw',3); plot_title('Local Z-rotation'); drawnow; record_vid(vid_obj);
end
end_vid_record(vid_obj);

%% 3. Coordinate Transforms
ccc
T_world_coord = pr2t([0,0,0],eye(3,3)); % 'world coord'
% Randomly determine the initial 'local coord' and 'T_a_local'
T_w2a_coord_init = pr2t(1.0*rand(1,3),rpy2r(360*rand(1,3))); % initital 'local coord'
T_a_p = pr2t([0,0,-1],rpy2r(360*rand(1,3))); % position in the 'a coord'

% Set figure
vid_obj = init_vid_record('../vid/ut03_coordinate_transform.mp4','HZ',40,'SAVE_VID',1);
set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',2.5*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_T(T_world_coord,'subfig_idx',1,'alen',1.0,'alw',4,'PLOT_AXIS_TIP',1,'atipsize',0.05,...
    'PLOT_SPHERE',1,'sr',0.05,'text_str','World Coord','text_fs',20); % 'world coord'
plot_T(T_w2a_coord_init,'subfig_idx',2,'alen',0.5,'alw',2,'als','--','PLOT_AXIS_TIP',0,...
    'PLOT_SPHERE',1,'sr',0.1,'text_str',''); % 'local coord'
for tick = 1:1:360
    % 1. Change the 'local coord' by rotating w.r.t. the x-axis
    T_w2a_coord = T_w2a_coord_init*pr2t([0,0,0],rpy2r(tick*[1,0,0]));
    % Pre-multiply the 'local coord' to transfrom 'T_a_local' to the 'world coord'
    T_a = T_w2a_coord * T_a_p;
    % Animate
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
end
end_vid_record(vid_obj);

%% 4. Plot Cube in 3D
ccc

T_world_coord = pr2t([0,0,0],eye(3,3)); % 'world coord'
T_cube_init = pr2t(2*rand(1,3),rpy2r(360*rand(1,3))); % 'box coord'

% Set figure
vid_obj = init_vid_record('../vid/ut04_plot_cube_3d.mp4','HZ',40,'SAVE_VID',1);
set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',5.0*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_T(T_world_coord,'subfig_idx',1,'alen',1.0,'alw',4,'PLOT_AXIS_TIP',1,'atipsize',0.05,...
    'PLOT_SPHERE',1,'sr',0.05,'text_str','World Coord','text_fs',20); % plot 'world coord'
plot_T(T_cube_init,'subfig_idx',2,'alen',0.5,'alw',3,'als','--','PLOT_AXIS_TIP',1,'atipsize',0.15,...
    'PLOT_SPHERE',1,'sr',0.05,'text_str','Box Coord','text_fs',20); % plot 'box coord'
plot_title('Rotate Cube w.r.t Local Z-axis');
for tick = 1:360
    T_cube = T_cube_init*pr2t([0,0,0],rpy2r(tick*[0,0,1])); % local z-rotation
    plot_line(t2p(T_world_coord),t2p(T_cube),'subfig_idx',1,...
        'color','k','lw',2,'ls','--','text_str','w2l'); % line from 'world coord' to 'box coord'
    plot_T(T_cube,'subfig_idx',3,'alen',0.5,'alw',3,'PLOT_AXIS_TIP',1,'atipsize',0.15,...
        'PLOT_SPHERE',1,'sr',0.05,'text_str','Box Coord','text_fs',20); % 'box coord'
    plot_cube(T_cube,[0,0,0],[1,2,3],'color','b','alpha',0.2,'edge_color','k'); % plot blue cube
    plot_title(sprintf('[%d/%d]',tick,360));
    drawnow; record_vid(vid_obj);
end
end_vid_record(vid_obj);

%% 5. Plot 3D arrow
ccc

% Position of the tip of the arrow
p_tip_init = [0.5,1.0,1.5];
T_world_coord = pr2t([0,0,0],eye(3,3)); % 'world coord'

% Set figure
vid_obj = init_vid_record('../vid/ut05_plot_arrow_3d.mp4','HZ',40,'SAVE_VID',1);
set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',2.0*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_T(T_world_coord,'subfig_idx',1,'alen',1.0,'alw',4,'PLOT_AXIS_TIP',1,'atipsize',0.05,...
    'PLOT_SPHERE',1,'sr',0.05,'text_str','World Coord','text_fs',20); % plot 'world coord'
for tick = 1:360
    T_tip = pr2t([0,0,0],rpy2r(tick*[0,0,1]))*pr2t(p_tip_init,eye(3,3));
    p_tip = t2p(T_tip);
    plot_arrow_3d([0,0,0],p_tip,'color','b','alpha',0.4); % plot arrow
    plot_T(T_tip,'subfig_idx',2,'PLOT_AXIS',0,'PLOT_AXIS_TIP',0,...
        'PLOT_SPHERE',1,'sr',0.05,'text_str','Arrow Tip','text_fs',20);
    plot_title(sprintf('[%d/%d]',tick,360));
    drawnow; record_vid(vid_obj);
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
chain.joint(1) = struct('name','world','parent',[],'childs',[2],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[0,0,0]','p_offset',[1,0,0]','R_offset',rpy2r([0,0,0]));
chain.joint(2) = struct('name','j1','parent',[1],'childs',[3],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[0,0,1]','p_offset',[1,0,0]','R_offset',rpy2r([0,-90,0]));
chain.joint(3) = struct('name','j2','parent',[2],'childs',[4],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[0,0,1]','p_offset',[1,0,0]','R_offset',rpy2r([0,0,0]));
chain.joint(4) = struct('name','j3','parent',[3],'childs',[5],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[0,0,1]','p_offset',[1,0,0]','R_offset',rpy2r([0,0,0]));
chain.joint(5) = struct('name','ee','parent',[4],'childs',[],'p',[0,0,0]','R',eye(3,3),...
    'q',0,'a',[0,0,0]','p_offset',[1,0,0]','R_offset',rpy2r([0,0,0]));
chain.n_joint = length(chain.joint);
chain.n_revolute_joint = 2;
chain.joint_names = {'world','j1','j2','j3','ee'};

vid_obj = init_vid_record('../vid/ut06_kinematic_chain_fk.mp4','HZ',40,'SAVE_VID',1); % init video

max_tick = 360;
traj = nan*ones(max_tick,3);
for tick = 1:max_tick
    
    % Update position in degree
    qs = tick*[1,2,4];
    chain = update_chain_q(chain,{'j1','j2','j3'},qs);
    
    % Forward kinematics
    chain = fk_chain(chain);
    
    % Plot the kinematic chain
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',[-0.5,2,-3,+3,-4,+4],'SET_DRAGZOOM',1,'GRID_ON',1);
    for i_idx = 1:chain.n_joint % plot links
        joint_i = chain.joint(i_idx);
        parent = joint_i.parent;
        if ~isempty(parent)
            joint_fr = chain.joint(parent);
            T_fr = pr2t(joint_fr.p,joint_fr.R);
            T_to = pr2t(joint_i.p,joint_i.R);
            plot_line(t2p(T_fr),t2p(T_to),'subfig_idx',i_idx,'color','k','ls','--');
        end
    end % for i_idx = 1:chain.n_joint % plot links
    for i_idx = 1:chain.n_joint % plot axis
        joint_i = chain.joint(i_idx);
        p_i = joint_i.p;
        R_i = joint_i.R;
        a_i = joint_i.a; % axis
        if sum(a_i) ~= 0
            axis_i = R_i*a_i;
            plot_arrow_3d(p_i,p_i+axis_i*0.5,'subfig_idx',i_idx,'color','b','sw',0.05,'tw',0.1);
        end
    end % for i_idx = 1:chain.n_joint % plot axis
    for i_idx = 1:chain.n_joint % plot joints
        joint_i = chain.joint(i_idx);
        T_i = pr2t(joint_i.p,joint_i.R);
        text_str = sprintf('[%d]%s',i_idx,joint_i.name);
        plot_T(T_i,'subfig_idx',i_idx,'alen',0.5,'alw',3,'PLOT_AXIS_TIP',0,...
            'PLOT_SPHERE',1,'sr',0.05,'sfc','k','sfa',0.7,'text_str',text_str,'text_fs',13);
    end % for i_idx = 1:chain.n_joint % plot joints
    % End-effector box
    T_ee = get_chain_T(chain,'ee');
    plot_cube(T_ee,[0.0,-0.2,-0.2],[1.0,0.4,0.4],'color','b','alpha',0.2,'edge_color','k');
    % End-effector trajectory
    traj(tick,:) = t2p(T_ee)';
    plot_curve_3d(traj(:,1),traj(:,2),traj(:,3),'color',0.5*[1,1,1]);
    % Title
    plot_title(sprintf('[%d/%d]',tick,max_tick));
    drawnow; record_vid(vid_obj);
    if ~ishandle(fig),break;end  % if figure is closed, then stop
end
end_vid_record(vid_obj);

%% 7. Get a Transformation Matrix from Three Points in 3D
ccc

% Three points in 3D
p1 = -1+2*rand(3,1); p2 = -1+2*rand(3,1); p3_init = -1+2*rand(3,1);

vid_obj = init_vid_record('../vid/ut07_transform_from_three_points.mp4','HZ',40,'SAVE_VID',1); % init video

for tick = 1:360
    % Rotate p3
    p3 = rpy2r(tick*[0,0,1]')*p3_init;
    % Get a transformation matrix from Three Points
    T = ps2T(p1,p2,p3);
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
    plot_title(sprintf('[%d/%d]',tick,360));
    drawnow; record_vid(vid_obj); 
    if ~ishandle(fig),break;end  % if figure is closed, then stop
end
end_vid_record(vid_obj);

%% 8. Plot Interactive Markers
ccc
global g_im

% Plot interactive markers
vid_obj = init_vid_record('../vid/ut08_interactive_markers.mp4','HZ',20,'SAVE_VID',1); % init video
fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',5*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_interactive_marker('subfig_idx',1,'T',pr2t([0,-2,0],rpy2r(0*rand(1,3))));
plot_interactive_marker('subfig_idx',2,'T',pr2t([0,+2,0],rpy2r(0*rand(1,3))));
tick = 0;
while ishandle(fig)
    tick = tick + 1;
    title_str = sprintf('[%d] p1:%s p2:%s \n',...
        tick,vec2str(t2p(g_im{1}.T)','%.2f'),vec2str(t2p(g_im{2}.T)','%.2f'));
    plot_title(title_str);
    drawnow limitrate; if ishandle(fig), record_vid(vid_obj); end
end
end_vid_record(vid_obj);

%% 9. Interpolate between Two Rotation Matrices, R1 and R2
ccc

R1 = rpy2r(360*rand(1,3));
R2 = rpy2r(360*rand(1,3));

w_deg = r2w(R1'*R2); % angular velocity in degree
w_hat = scew(w_deg*pi/180); % make sure to use radian here

vid_obj = init_vid_record('../vid/ut09_interpolate_rotations.mp4','HZ',20); % init video

for t = linspace(0,1,100) 
    R_t = R1*expm(w_hat*t);
    % Plot
    fig = set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',[80,16],'axis_info',3*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
    plot_T(r2t(R1),'subfig_idx',1,'text_str','R1','TEXT_AT_ZTIP',1);  
    plot_T(r2t(R2),'subfig_idx',2,'text_str','R2','TEXT_AT_ZTIP',1);  
    plot_T(r2t(R_t),'subfig_idx',3,'alw',3,'PLOT_AXIS_TIP',1,...
        'text_str','R_t','TEXT_AT_ZTIP',1,'text_fs',20,'text_color','b');  
    plot_title(sprintf('t:[%.2f]',t));
    drawnow; record_vid(vid_obj);
end
end_vid_record(vid_obj);

%%























































