function plot_skeleton_from_features(x,varargin)
%
% Plot skeleton from 21 dimension features of 7 unit vectors
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'fig_pos',[0.0,0.4,0.35,0.5]); % figure position
addParameter(p,'color','k');
addParameter(p,'colors','');
addParameter(p,'joi_sr',0.05);
addParameter(p,'lw',2);
addParameter(p,'ls','-');
addParameter(p,'axis_info','');
addParameter(p,'view_info',[88,16]);

addParameter(p,'d_r2n',1.0); % root to neck 
addParameter(p,'d_n2s',0.3); % neck to shoulder dist
addParameter(p,'d_s2e',0.5); % shoulder to elbow
addParameter(p,'d_e2h',0.5); % elbow to hand

parse(p,varargin{:});

fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
fig_pos = p.Results.fig_pos;
color = p.Results.color;
colors = p.Results.colors;
joi_sr = p.Results.joi_sr;
lw = p.Results.lw;
ls = p.Results.ls;
axis_info = p.Results.axis_info;
view_info = p.Results.view_info;

d_r2n = p.Results.d_r2n;
d_n2s = p.Results.d_n2s;
d_s2e = p.Results.d_s2e;
d_e2h = p.Results.d_e2h;

% Parse skeleton featrue
r2n    = x(1:3);
n2rs   = x(4:6);
rs2re  = x(7:9);
re2rh  = x(10:12);
n2ls   = x(13:15);
ls2le  = x(16:18);
le2lh  = x(19:21);

% Reconstruct skeleton
p_root = [0,0,0];
p_neck = p_root + d_r2n*r2n;
p_rs = p_neck + d_n2s*n2rs;
p_re = p_rs + d_s2e*rs2re;
p_rh = p_re + d_e2h*re2rh;
p_ls = p_neck + d_n2s*n2ls;
p_le = p_ls + d_s2e*ls2le;
p_lh = p_le + d_e2h*le2lh;

d_r2n = norm(p_root-p_neck);
p_rp = p_root + 0.35*d_r2n*[0,-1,0]; % right pelvis
p_lp = p_root + 0.35*d_r2n*[0,1,0]; % left pelvis
p_rf = p_rp + 1.4*d_r2n*[0,0,-1]; % right foot
p_lf = p_lp + 1.4*d_r2n*[0,0,-1]; % left foot


plot_lines(...
    [p_root;p_neck;p_rs;p_re;p_neck;p_ls;p_le;p_root;p_rp;p_root;p_lp],...
    [p_neck;p_rs;  p_re;p_rh;p_ls;  p_le;p_lh;p_rp;  p_rf;p_lp;  p_lf],...
    'fig_idx',fig_idx,'subfig_idx',subfig_idx,'fig_pos',fig_pos,...
    'color',color,'colors',colors,'lw',lw,'ls',ls);

plot_spheres([p_rs;p_re;p_rh;p_ls;p_le;p_lh;p_neck],...
    'fig_idx',fig_idx,'subfig_idx',subfig_idx,'sr',joi_sr,'sfc',color,'sfa',0.8);

if h{fig_idx,subfig_idx}.first_flag 
    h{fig_idx,subfig_idx}.first_flag = false;
    
    % axis equal;
    if ~isempty(axis_info)
        axis(axis_info);
    end
    
    if ~isempty(view_info)
        view(view_info(1),view_info(2));
    end
    % grid on;
    dragzoom;
else
    
end





