function plot_grp1d(grp1d,sampled_paths,varargin)
%
% Plot GRP 1D
%

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'lw_sample',1); % line width of sampled paths
addParameter(p,'axis_info','');
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
lw_sample = p.Results.lw_sample;
axis_info = p.Results.axis_info;

% # sampled paths
n_path = size(sampled_paths,2);

% Plot 
fig = figure(fig_idx);
set_fig_position(fig,'position',[0.0,0.4,0.5,0.4],'AXIS_EQUAL',0,'SET_DRAGZOOM',0,'AXES_LABEL',0);
h_fill = fill([grp1d.t_test;...
    flipud(grp1d.t_test)],[grp1d.mu_test-2*grp1d.std_test ;...
    flipud(grp1d.mu_test+2*grp1d.std_test)],...
    'k','LineStyle',':'); % grp 2std
set(h_fill,'FaceAlpha',0.2);
colors = linspecer(n_path);
for i_idx = 1:n_path
    sampled_path = sampled_paths(:,i_idx);
    color = colors(i_idx,:); % 0.4*[1,1,1];
    plot(grp1d.t_test,sampled_path,'-','linewidth',lw_sample,'color',color);
end
h_anchor = plot(grp1d.t_anchor,grp1d.x_anchor,'ko','linewidth',3,'markersize',15);
h_mu = plot(grp1d.t_test,grp1d.mu_test,'k-','linewidth',4);
legend([h_anchor,h_mu,h_fill],{'Data','GP mu','GP std'},'fontsize',20,'location','southeast');

if ~isempty(axis_info)
    axis (axis_info);
end
