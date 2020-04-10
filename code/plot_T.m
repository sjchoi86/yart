function plot_T(T,varargin)
%
% Plot Homogeneous Transformation Matrix
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'PLOT_AXIS',true);
addParameter(p,'alen',1.0); % axis length
addParameter(p,'alw',2); % axis line width
addParameter(p,'als','-'); % axis line style
addParameter(p,'PLOT_AXIS_TIP',true);
addParameter(p,'tipsize',0.1); % axis tip size w.r.t. alen
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
PLOT_AXIS = p.Results.PLOT_AXIS;
alen = p.Results.alen;
alw = p.Results.alw;
als = p.Results.als;
PLOT_AXIS_TIP = p.Results.PLOT_AXIS_TIP;
tipsize = p.Results.tipsize;

% Get p and R
p = T([1,2,3],4);
R = T([1,2,3],[1,2,3]);
ex = R(:,1); ey = R(:,2); ez = R(:,3);


% Plot start here
if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    hold on;
    
    if PLOT_AXIS
        h{fig_idx,subfig_idx}.cx = plot3([p(1),p(1)+alen*ex(1)],[p(2),p(2)+alen*ex(2)],...
            [p(3),p(3)+alen*ex(3)],'r','LineWidth',alw,'LineStyle',als);
        h{fig_idx,subfig_idx}.cy = plot3([p(1),p(1)+alen*ey(1)],[p(2),p(2)+alen*ey(2)],...
            [p(3),p(3)+alen*ey(3)],'g','LineWidth',alw,'LineStyle',als);
        h{fig_idx,subfig_idx}.cz = plot3([p(1),p(1)+alen*ez(1)],[p(2),p(2)+alen*ez(2)],...
            [p(3),p(3)+alen*ez(3)],'b','LineWidth',alw,'LineStyle',als);
    end
    
    if PLOT_AXIS_TIP
        [xs,ys,zs] = sphere(20);
        fv = surf2patch(tipsize*alen*xs,tipsize*alen*ys,tipsize*alen*zs);
        
        ax = p(1)+alen*ex(1);
        ay = p(2)+alen*ex(2);
        az = p(3)+alen*ex(3);
        h{fig_idx,subfig_idx}.sx = patch(fv,...
            'EdgeColor','none','FaceColor','r','FaceAlpha',0.9,'facelighting','gouraud');
        h{fig_idx,subfig_idx}.sx_t = hgtransform;
        set(h{fig_idx,subfig_idx}.sx,'parent',h{fig_idx,subfig_idx}.sx_t);
        tform = pr2t([ax,ay,az]);
        set(h{fig_idx,subfig_idx}.sx_t,'Matrix',tform);
        
        ax = p(1)+alen*ey(1);
        ay = p(2)+alen*ey(2);
        az = p(3)+alen*ey(3);
        h{fig_idx,subfig_idx}.sy = patch(fv,...
            'EdgeColor','none','FaceColor','g','FaceAlpha',0.9,'facelighting','gouraud');
        h{fig_idx,subfig_idx}.sy_t = hgtransform;
        set(h{fig_idx,subfig_idx}.sy,'parent',h{fig_idx,subfig_idx}.sy_t);
        tform = pr2t([ax,ay,az]);
        set(h{fig_idx,subfig_idx}.sy_t,'Matrix',tform);
        
        ax = p(1)+alen*ez(1);
        ay = p(2)+alen*ez(2);
        az = p(3)+alen*ez(3);
        h{fig_idx,subfig_idx}.sz = patch(fv,...
            'EdgeColor','none','FaceColor','b','FaceAlpha',0.9,'facelighting','gouraud');
        h{fig_idx,subfig_idx}.sz_t = hgtransform;
        set(h{fig_idx,subfig_idx}.sz,'parent',h{fig_idx,subfig_idx}.sz_t);
        tform = pr2t([ax,ay,az]);
        set(h{fig_idx,subfig_idx}.sz_t,'Matrix',tform);
    end
    
    
else
    
    if PLOT_AXIS
        h{fig_idx,subfig_idx}.cx.XData = [p(1),p(1)+alen*ex(1)];
        h{fig_idx,subfig_idx}.cx.YData = [p(2),p(2)+alen*ex(2)];
        h{fig_idx,subfig_idx}.cx.ZData = [p(3),p(3)+alen*ex(3)];
        h{fig_idx,subfig_idx}.cy.XData = [p(1),p(1)+alen*ey(1)];
        h{fig_idx,subfig_idx}.cy.YData = [p(2),p(2)+alen*ey(2)];
        h{fig_idx,subfig_idx}.cy.ZData = [p(3),p(3)+alen*ey(3)];
        h{fig_idx,subfig_idx}.cz.XData = [p(1),p(1)+alen*ez(1)];
        h{fig_idx,subfig_idx}.cz.YData = [p(2),p(2)+alen*ez(2)];
        h{fig_idx,subfig_idx}.cz.ZData = [p(3),p(3)+alen*ez(3)];
    end
    
    if PLOT_AXIS_TIP
        ax = p(1)+alen*ex(1);
        ay = p(2)+alen*ex(2);
        az = p(3)+alen*ex(3);
        tform = pr2t([ax,ay,az]);
        set(h{fig_idx,subfig_idx}.sx_t,'Matrix',tform);
        
        ax = p(1)+alen*ey(1);
        ay = p(2)+alen*ey(2);
        az = p(3)+alen*ey(3);
        tform = pr2t([ax,ay,az]);
        set(h{fig_idx,subfig_idx}.sy_t,'Matrix',tform);
        
        ax = p(1)+alen*ez(1);
        ay = p(2)+alen*ez(2);
        az = p(3)+alen*ez(3);
        tform = pr2t([ax,ay,az]);
        set(h{fig_idx,subfig_idx}.sz_t,'Matrix',tform);
    end
    
    
end














