function [x,c] = get_synthetic_2d_point(varargin)

% Parse options
p = inputParser;
addParameter(p,'xmin',-1);
addParameter(p,'xmax',+1);
addParameter(p,'ymin',-1);
addParameter(p,'ymax',+1);
addParameter(p,'xres',0.05);
addParameter(p,'yres',0.05);
addParameter(p,'PERMUTE',0);
addParameter(p,'append_rate',0);
addParameter(p,'x0',0.0);
addParameter(p,'y0',0.0);
parse(p,varargin{:});

xmin = p.Results.xmin;
xmax = p.Results.xmax;
ymin = p.Results.ymin;
ymax = p.Results.ymax;

xres = p.Results.xres;
yres = p.Results.yres;
PERMUTE = p.Results.PERMUTE;
append_rate = p.Results.append_rate;
x0 = p.Results.x0;
y0 = p.Results.y0;

% Grid
[xs,ys] = meshgrid(xmin:xres:xmax,ymin:yres:ymax);
xys = [xs(:),ys(:)];
x = zeros(size(xys));
n_cnt = 0;
for i_idx = 1:size(xys,1)
    xy = xys(i_idx,:);
    n_cnt = n_cnt + 1;
    x(n_cnt,:) = xy;
end
x = x(1:n_cnt,:);
% Append
n_x = size(x,1); 
n_append = round(n_x*append_rate);
x_append = [x0,y0] + rand(n_append,2);
x = [x; x_append];
n = size(x,1);
% Permute
if PERMUTE
    perm_idx = randperm(n);
    x = x(perm_idx,:);
end
% Color
c = get_color_with_first_and_second_coordinates(x);











