function fig = plot_img(img,varargin)
%
% Plot images
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'fig_pos','');
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
fig_pos = p.Results.fig_pos;


% Plot start here
if h{fig_idx,subfig_idx}.first_flag || (~ishandle(h{fig_idx,subfig_idx}.fig))
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    if ~isempty(fig_pos)
        set_fig_position(h{fig_idx,subfig_idx}.fig,'position',fig_pos,...
            'SET_DRAGZOOM',0,'AXIS_EQUAL',0,'AXES_LABEL',0,'GRID_ON',0,...
            'SET_LIGHT',0);
    end
    h{fig_idx,subfig_idx}.image = imshow(img);
else
    h{fig_idx,subfig_idx}.image.CData = img;
end

fig = h{fig_idx,subfig_idx}.fig;

