ccc
%
% Here, we introduce some basic unit functions that will be used throughout this package
%
% 1. Make figures with different positions
% 2. Plot Homogeneous Transformation Matrices
%
%% 1. Make figures with different positions
ccc

set_fig_position(figure(1),'position',[0.0,0.6,0.2,0.35],'ADD_TOOLBAR',1);
set_fig_position(figure(2),'position',[0.2,0.6,0.2,0.35],'ADD_TOOLBAR',1);
set_fig_position(figure(3),'position',[0.4,0.6,0.2,0.35],'ADD_TOOLBAR',1);

%% 2. Plot Homogeneous Transformation Matrices
% pre-multiply: global transfrom / post-multiply: local transform
ccc

T_init = pr2t([0,0,0],rpy2r(360*rand(1,3)));

% Plot figure
set_fig_position(figure(1),'position',[0.5,0.6,0.2,0.35],'ADD_TOOLBAR',1);
view(80,16); axis(2*[-1,+1,-1,+1,-1,+1]); grid_on('color','k','alpha',0.9); 
dragzoom; % drag zoom

plot_T(T_init,'subfig_idx',2,'alw',1,'als','--','PLOT_AXIS_TIP',0); % plot 
for tick = 1:5:360 % Global rotate w.r.t. x-axis
    T = pr2t([0,0,0],rpy2r(tick*[1,0,0]))*T_init; 
    plot_T(T,'alw',3); title('Global X-rotation','fontsize',25,'fontname','consolas'); drawnow;
end
for tick = 1:5:360 % Global rotate w.r.t. y-axis
    T = pr2t([0,0,0],rpy2r(tick*[0,1,0]))*T_init; 
    plot_T(T,'alw',3); title('Global Y-rotation','fontsize',25,'fontname','consolas'); drawnow;
end
for tick = 1:5:360 % Global rotate w.r.t. z-axis
    T = pr2t([0,0,0],rpy2r(tick*[0,0,1]))*T_init; 
    plot_T(T,'alw',3); title('Global Z-rotation','fontsize',25,'fontname','consolas'); drawnow;
end
for tick = 1:5:360 % Local rotate w.r.t. x-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[1,0,0])); 
    plot_T(T,'alw',3); title('Local X-rotation','fontsize',25,'fontname','consolas'); drawnow;
end
for tick = 1:5:360 % Local rotate w.r.t. y-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[0,1,0])); 
    plot_T(T,'alw',3); title('Local Y-rotation','fontsize',25,'fontname','consolas'); drawnow;
end
for tick = 1:5:360 % Local rotate w.r.t. z-axis
    T = T_init*pr2t([0,0,0],rpy2r(tick*[0,0,1])); 
    plot_T(T,'alw',3); title('Local Z-rotation','fontsize',25,'fontname','consolas'); drawnow;
end

%%
ccc

