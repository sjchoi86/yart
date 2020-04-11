ccc
%
% Here, we introduce some basic unit functions that will be used throughout this package
%
% 1. Make figures with different positions
% 2. Plot Homogeneous Transformation Matrices (Pre:global / Post:local)
% 3. Coordinate Transforms
% 4. Plot Cube in 3D
% 5. Plot 3D arrow
%
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
set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',2.0*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_T(T_init,'subfig_idx',2,'alw',1,'als','--','PLOT_AXIS_TIP',0); % plot initial T
plot_T(pr2t([0,0,0],eye(3,3)),'subfig_idx',3,'alen',1.0,'alw',3,'PLOT_AXIS_TIP',0,...
    'PLOT_SPHERE',0); % plot origin
for tick = 1:5:360 % Global rotate w.r.t. x-axis
    T = pr2t([0,0,0],rpy2r(tick*[1,0,0]))*T_init; % pre-mul
    plot_T(T,'alw',3); plot_title('Global X-rotation'); drawnow;
end
for tick = 1:5:360 % Global rotate w.r.t. y-axis
    T = pr2t([0,0,0],rpy2r(tick*[0,1,0]))*T_init; % pre-mul
    plot_T(T,'alw',3); plot_title('Global Y-rotation'); drawnow;
end
for tick = 1:5:360 % Global rotate w.r.t. z-axis
    T = pr2t([0,0,0],rpy2r(tick*[0,0,1]))*T_init; % pre-mul
    plot_T(T,'alw',3); plot_title('Global Z-rotation'); drawnow;
end
for tick = 1:5:360 % Local rotate w.r.t. x-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[1,0,0])); % post-mul
    plot_T(T,'alw',3); plot_title('Local X-rotation'); drawnow;
end
for tick = 1:5:360 % Local rotate w.r.t. y-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[0,1,0])); % post-mul
    plot_T(T,'alw',3); plot_title('Local Y-rotation'); drawnow;
end
for tick = 1:5:360 % Local rotate w.r.t. z-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[0,0,1])); % post-mul
    plot_T(T,'alw',3); plot_title('Local Z-rotation'); drawnow;
end

%% 3. Coordinate Transforms
ccc
T_world_coord = pr2t([0,0,0],eye(3,3)); % 'world coord'
% Randomly determine the initial 'local coord' and 'T_a_local'
T_local_init = pr2t(-1.0+2.0*rand(1,3),rpy2r(360*rand(1,3))); % 'local coord'
T_a_local = pr2t([0,0,-1],rpy2r(360*rand(1,3))); % local position in the 'local coord'

% Set figure
set_fig_position(figure(1),'position',[0.5,0.5,0.2,0.35],'ADD_TOOLBAR',1,'AXES_LABEL',1,...
    'view_info',[80,16],'axis_info',2.5*[-1,+1,-1,+1,-1,+1],'SET_DRAGZOOM',1,'GRID_ON',1);
plot_T(T_world_coord,'subfig_idx',1,'alen',1.0,'alw',4,'PLOT_AXIS_TIP',1,'atipsize',0.05,...
    'PLOT_SPHERE',1,'sr',0.05,'text_str','World Coord','text_fs',20); % 'world coord'
for tick = 1:1:360*5
    % 1. Change the 'local coord' by rotating w.r.t. the x-axis
    T_local_coord = T_local_init*pr2t([0,0,0],rpy2r(tick*[1,0,0]));
    % Pre-multiply the 'local coord' to transfrom 'T_a_local' to the 'world coord'
    T_a_world = T_local_coord * T_a_local;
    % Animate
    plot_line(t2p(T_world_coord),t2p(T_local_coord),'subfig_idx',1,...
        'color','k','lw',2,'ls','--','text_str','w2l'); % line from 'world coord' to 'local coord'
    plot_T(T_local_coord,'subfig_idx',2,'alen',0.5,'alw',3,'PLOT_AXIS_TIP',1,'atipsize',0.1,...
        'PLOT_SPHERE',1,'sr',0.1,'text_str','Local Coord','text_fs',13); % 'local coord'
    plot_line(t2p(T_local_coord),t2p(T_a_world),'subfig_idx',2,...
        'color','b','lw',1,'ls','--','text_str','l2a'); % line from 'local coord' to 'p_a'
    plot_T(T_a_world,'subfig_idx',3,'alen',0.3,'alw',3,'atipsize',0.2,...
        'PLOT_SPHERE',1,'sr',0.1,'text_str','T_a'); % T_a in the 'world coord'
    drawnow;
end

%% 4. Plot Cube in 3D
ccc

T_world_coord = pr2t([0,0,0],eye(3,3)); % 'world coord'
T_cube_init = pr2t(2*rand(1,3),rpy2r(360*rand(1,3))); % 'box coord'

% Set figure
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
    drawnow;
end

%% 5. Plot 3D arrow
ccc

% Position of the tip of the arrow
p_tip_init = [0.5,1.0,1.5];
T_world_coord = pr2t([0,0,0],eye(3,3)); % 'world coord'

% Set figure
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
    drawnow;
end

%% 6.













