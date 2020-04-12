function plot_line(p1,p2,varargin)
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
addParameter(p,'text_str','');
addParameter(p,'text_fs',13);
addParameter(p,'text_fn','consolas');
addParameter(p,'text_color','k');
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
color = p.Results.color;
lw = p.Results.lw;
ls = p.Results.ls;
text_str = p.Results.text_str;
text_fs = p.Results.text_fs;
text_fn = p.Results.text_fn;
text_color = p.Results.text_color;

% Plot start here
if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    hold on;
    
    h{fig_idx,subfig_idx}.line = plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],...
        color,'LineWidth',lw,'LineStyle',ls);
    
    if ~isempty(text_str)
        p = 0.5*(p1+p2);
        h{fig_idx,subfig_idx}.text = text(p(1),p(2),p(3),[' ',text_str],...
            'FontSize',text_fs,'FontName',text_fn,'Color',text_color,'Interpreter','none');
    end
    
else
    
    h{fig_idx,subfig_idx}.line.XData = [p1(1),p2(1)];
    h{fig_idx,subfig_idx}.line.YData = [p1(2),p2(2)];
    h{fig_idx,subfig_idx}.line.ZData = [p1(3),p2(3)];
    
    if ~isempty(text_str)
        p = 0.5*(p1+p2);
        h{fig_idx,subfig_idx}.text.Position = p;
        h{fig_idx,subfig_idx}.text.String = [' ',text_str];
    end
    
end
