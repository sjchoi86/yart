function plot_arrow_3d(p1,p2,varargin)
%
% Plot 3D arrow
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse input arguments
len = norm(p1-p2);
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'alpha',0.5);
addParameter(p,'color','r');
addParameter(p,'sw',len/20); % stem width
addParameter(p,'tw',len/10); % tip width
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
alpha = p.Results.alpha;
color = p.Results.color;
sw = p.Results.sw;
tw = p.Results.tw;

% Get 3D arrow mesh
fv = get_arrow_3d(p1,p2,'color',color,'stemWidth',sw,'tipWidth',tw,'facealpha',alpha);

if h{fig_idx,subfig_idx}.first_flag || (~ishandle(h{fig_idx,subfig_idx}.fig))
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    % Plot arrow
    h{fig_idx,subfig_idx}.arrow = patch(fv,'facecolor',color,'edgeColor','none',...
        'FaceAlpha',alpha,'FaceLighting','gouraud');
else
    h{fig_idx,subfig_idx}.arrow.Faces = fv.faces;
    h{fig_idx,subfig_idx}.arrow.Vertices = fv.vertices;
end
