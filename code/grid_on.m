function grid_on(varargin)

% Parse options
p = inputParser;
addParameter(p,'color','k');
addParameter(p,'alpha',0.5);
parse(p,varargin{:});
color = p.Results.color; % grid line color
alpha = p.Results.alpha; % grid line alpha

% Grid on
grid on;

% Set other properties
ax = gca; 
ax.GridColor = color;
ax.GridAlpha = alpha;
