function plot_lines(p1s,p2s,varargin)
%
% Plot lines
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

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

n = size(p1s,1);
if isempty(colors)
    colors = linspecer(n);
end

% Plot start here
if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    hold on;
    
    for i_idx = 1:n
        p1 = p1s(i_idx,:);
        p2 = p2s(i_idx,:);
        if isempty(color)
            color_i = colors(i_idx,:);
        else
            color_i = color;
        end
        h{fig_idx,subfig_idx}.line{i_idx} = ...
            plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],...
            'Color',color_i,'LineWidth',lw,'LineStyle',ls);
    end
    
else
    
    for i_idx = 1:n
        p1 = p1s(i_idx,:);
        p2 = p2s(i_idx,:);
        
        h{fig_idx,subfig_idx}.line{i_idx}.XData = [p1(1),p2(1)];
        h{fig_idx,subfig_idx}.line{i_idx}.YData = [p1(2),p2(2)];
        h{fig_idx,subfig_idx}.line{i_idx}.ZData = [p1(3),p2(3)];
    end
    
end




