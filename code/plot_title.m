function plot_title(title_str,varargin)
%
% Plot a title
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'fs',20);
addParameter(p,'fn','consolas');
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
fs = p.Results.fs;
fn = p.Results.fn;


% Plot start here
if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    
    h{fig_idx,subfig_idx}.title = title(title_str,'fontsize',fs,'fontname',fn);
    
else
    
    h{fig_idx,subfig_idx}.title.String = title_str;
end
