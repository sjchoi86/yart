function plot_curve_3d(x,y,z,varargin)
%
% Plot a straight line between two points p1 and p2 in 3D
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'color','k');
addParameter(p,'lw',2);
addParameter(p,'ls','-');
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
color = p.Results.color;
lw = p.Results.lw;
ls = p.Results.ls;

% Plot start here
if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    
    h{fig_idx,subfig_idx}.line = plot3(x,y,z,...
        'Color',color,'LineWidth',lw,'LineStyle',ls);
    
else
    
    h{fig_idx,subfig_idx}.line.XData = x;
    h{fig_idx,subfig_idx}.line.YData = y;
    h{fig_idx,subfig_idx}.line.ZData = z;
    
end
