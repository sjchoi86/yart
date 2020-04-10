function fig = set_fig_position(fig,varargin)
%
% Set the position of a figure
%

% Parse options
p = inputParser;
addParameter(p,'position',[0.0,0.5,0.3,0.5]);
addParameter(p,'ADD_TOOLBAR',1);
parse(p,varargin{:});
position = p.Results.position;
ADD_TOOLBAR = p.Results.ADD_TOOLBAR;

% Current size of the screen
sz = get(0, 'ScreenSize');

% Set the position of a figure relative to the screen size
fig_pos = [position(1)*sz(3),position(2)*sz(4),position(3)*sz(3),position(4)*sz(4)];
set(fig,'Position',fig_pos);

% Add toolbar to a figure
if ADD_TOOLBAR
    addToolbarExplorationButtons(gcf);
end

% Disable toolbar at the upper right of axes
ax = gca;
ax.Toolbar = [];

% Axis equal
axis equal;