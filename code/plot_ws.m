function plot_ws(ws,varargin)
%
% Plot JOI workspaces
%

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'color','y');
addParameter(p,'alpha',0.1);
addParameter(p,'edge_color','k');
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
color = p.Results.color;
alpha = p.Results.alpha;
edge_color = p.Results.edge_color;


for w_idx = 1:ws.n
    plot_cube(p2t([0,0,0]),ws.info{w_idx}.min_vals,ws.info{w_idx}.len_vals,...
        'fig_idx',fig_idx,'subfig_idx',w_idx,...
        'color',color,'alpha',alpha,'edge_color',edge_color);
end