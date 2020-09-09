function plot_skeleton_from_joi(joi_positions,varargin)
%
% Plot a skeleton from JOI positions
%

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'color','');
addParameter(p,'colors','');
addParameter(p,'lw',2);
addParameter(p,'ls','-');
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
color = p.Results.color;
colors = p.Results.colors;
lw = p.Results.lw;
ls = p.Results.ls;


p_root = joi_positions(1,:)';
p_rs = joi_positions(2,:)';
p_re = joi_positions(3,:)';
p_rh = joi_positions(4,:)';
p_ls = joi_positions(5,:)';
p_le = joi_positions(6,:)';
p_lh = joi_positions(7,:)';
p_neck = joi_positions(8,:)';

plot_lines([p_root,p_neck,p_rs,p_re,p_neck,p_ls,p_le]',...
    [p_neck,p_rs,p_re,p_rh,p_ls,p_le,p_lh]',...
    'fig_idx',fig_idx,'subfig_idx',subfig_idx,'color',color,'colors',colors,...
    'lw',lw,'ls',ls);
