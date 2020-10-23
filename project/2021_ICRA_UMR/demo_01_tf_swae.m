addpath_code
ccc
%% 1. Parse TF-trained SWAE
ccc

% Parse
mat_path_a = 'script/nets/wae_x/weights.mat';
mat_path_b = 'script/nets/wae_y/weights.mat';
W_a = parse_wae(mat_path_a);
W_b = parse_wae(mat_path_b);

% Test
[x_data,c] = get_synthetic_2d_point(...
    'xmin',-0.5,'xmax',0.5,'ymin',-0.5,'ymax',0.5,'xres',0.01,'yres',0.01);
z_from_a = encode_wae(W_a,x_data);
x_a_to_b = decode_wae(W_b,z_from_a);

% Plot
xm = 0.1; ym = 0.1;
fig = set_fig_position(figure(1),'position',[0.0,0.5,0.5,0.4],...
    'DISABLE_TOOLBAR',0,'HOLD_ON',0,'SET_LIGHT',0,'AXIS_EQUAL',0,'SET_DRAGZOOM',0,...
    'GRID_ON',0,'AXES_LABEL',0);

subaxes(fig,1,2,1,xm,ym); hold on; % Train A
scatter(x_data(:,1),x_data(:,2),5,c,'filled','MarkerEdgeColor','none',...
    'MarkerFaceAlpha',0.5);
am = 0.2;
axis equal; axis([-1-am,+1+am,-1-am,+1+am]); grid on;

subaxes(fig,1,2,2,xm,ym); hold on; % Train A
scatter(x_a_to_b(:,1),x_a_to_b(:,2),5,c,'filled','MarkerEdgeColor','none',...
    'MarkerFaceAlpha',0.5);
axis equal; axis([-1-am,+1+am,-1-am,+1+am]); grid on;

%%

