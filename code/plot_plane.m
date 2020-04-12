function plot_plane(ps,varargin)
%
% Plot a plane composed of 3 or more points
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'fc','y');
addParameter(p,'ec','k');
addParameter(p,'alpha',0.5);
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
fc = p.Results.fc;
ec = p.Results.ec;
alpha = p.Results.alpha;

% Plot start here
if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    
    h{fig_idx,subfig_idx}.plane = patch('Faces',1:3,'Vertices',ps);
    set(h{fig_idx,subfig_idx}.plane,...
        'FaceColor',fc,'EdgeColor',ec,'LineWidth',2,'FaceAlpha',alpha);
    
else
    h{fig_idx,subfig_idx}.plane.Vertices = ps;
    
end

