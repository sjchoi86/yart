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
addParameter(p,'atipsize',0.1); % axis tip size w.r.t. alen
addParameter(p,'PLOT_SPHERE',true);
addParameter(p,'sr',0.1); % sphere radius
addParameter(p,'sfc',0.5*[1,1,1]); % sphere face color
addParameter(p,'sfa',0.5); % sphere face alpha
addParameter(p,'text_str','');
addParameter(p,'text_fs',15);
addParameter(p,'text_fn','consolas');
addParameter(p,'text_color','k');
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
PLOT_AXIS = p.Results.PLOT_AXIS;
alen = p.Results.alen;
alw = p.Results.alw;
als = p.Results.als;
PLOT_AXIS_TIP = p.Results.PLOT_AXIS_TIP;
atipsize = p.Results.atipsize;
PLOT_SPHERE = p.Results.PLOT_SPHERE;
sr = p.Results.sr;
sfc = p.Results.sfc;
sfa = p.Results.sfa;
text_str = p.Results.text_str;
text_fs = p.Results.text_fs;
text_fn = p.Results.text_fn;
text_color = p.Results.text_color;

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
        fv = surf2patch(atipsize*alen*xs,atipsize*alen*ys,atipsize*alen*zs);
        
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
    
    if PLOT_SPHERE
        [x,y,z] = ellipsoid(0,0,0,sr,sr,sr,30);
        fv = surf2patch(x,y,z);
        h{fig_idx,subfig_idx}.sphere = patch(fv,...
            'EdgeColor','none','FaceColor',sfc,'FaceAlpha',sfa,'facelighting','gouraud');
        h{fig_idx,subfig_idx}.sphere_t = hgtransform;
        set(h{fig_idx,subfig_idx}.sphere,'parent',h{fig_idx,subfig_idx}.sphere_t);
        tform = pr2t(p);
        set(h{fig_idx,subfig_idx}.sphere_t,'Matrix',tform);
    end
    
    if ~isempty(text_str)
        h{fig_idx,subfig_idx}.text = text(p(1),p(2),p(3),[' ',text_str],...
            'FontSize',text_fs,'FontName',text_fn,'Color',text_color,'Interpreter','none');
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
    
    if PLOT_SPHERE
        tform = pr2t(p);
        set(h{fig_idx,subfig_idx}.sphere_t,'Matrix',tform);
    end
    
    if ~isempty(text_str)
        h{fig_idx,subfig_idx}.text.Position = p;
        h{fig_idx,subfig_idx}.text.String = [' ',text_str];
    end
    
end














