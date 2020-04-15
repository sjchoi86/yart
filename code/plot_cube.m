function plot_cube(T,xyz_min,xyz_len,varargin)
%
% Plot cube
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'color','b');
addParameter(p,'alpha',0.5);
addParameter(p,'edge_color','none');
addParameter(p,'lw',1);
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
color = p.Results.color;
alpha = p.Results.alpha;
edge_color = p.Results.edge_color;
lw = p.Results.lw;

% Cube configuration
xyz_min = reshape(xyz_min,[1,3]);
xyz_len = reshape(xyz_len,[1,3]);
[p,R] = t2pr(T);
vertex_matrix = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
faces_matrix = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
vertex_matrix = vertex_matrix.*xyz_len;
vertex_matrix = vertex_matrix + xyz_min; % do basic translation

if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    
    % Plot cube
    h{fig_idx,subfig_idx}.patch = patch('Vertices',vertex_matrix,...
        'Faces',faces_matrix,...
        'FaceColor',color,'FaceAlpha',alpha,...
        'EdgeColor',edge_color,'lineWidth',lw); % 'EdgeColor','none');
    h{fig_idx,subfig_idx}.patch_t = hgtransform;
    set(h{fig_idx,subfig_idx}.patch,'parent',h{fig_idx,subfig_idx}.patch_t);
    tform = pr2t(p,R);
    set(h{fig_idx,subfig_idx}.patch_t,'Matrix',tform);
else
    tform = pr2t(p,R);
    set(h{fig_idx,subfig_idx}.patch_t,'Matrix',tform);
end
