function plot_joi_chain(chain,joi_chain,varargin)

persistent h
if isempty(h),for i_idx = 1:100,for j = 1:200,h{i_idx,j}.first_flag = 1;end;end;end

% Parse options
chain_sz = get_chain_sz(chain);
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'PLOT_COORD',0);
addParameter(p,'PLOT_COORD_TIP',0);
addParameter(p,'sr_rate',0.1);
addParameter(p,'PLOT_SPHERE',1);
addParameter(p,'PRINT_JOI_NAME',0);
addParameter(p,'tfs',15);
addParameter(p,'clen',max(chain_sz.xyz_len)/25);
addParameter(p,'clw',4);
addParameter(p,'sr',max(chain_sz.xyz_len)/25);
addParameter(p,'sfc','k');
addParameter(p,'sfa',0.5);
addParameter(p,'colors','');
addParameter(p,'joi_types','');
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
PLOT_COORD = p.Results.PLOT_COORD;
PLOT_COORD_TIP = p.Results.PLOT_COORD_TIP;
sr_rate = p.Results.sr_rate;
PLOT_SPHERE = p.Results.PLOT_SPHERE;
PRINT_JOI_NAME = p.Results.PRINT_JOI_NAME;
tfs = p.Results.tfs;
clen = p.Results.clen; % coordinate length
clw = p.Results.clw;
sr = p.Results.sr;
sfc = p.Results.sfc;
sfa = p.Results.sfa;
colors = p.Results.colors;
joi_types = p.Results.joi_types;


if h{fig_idx,subfig_idx}.first_flag || ...% first flag
        ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    
    % Get JOI indices
    if isempty(joi_types)
        joi_idxs = 1:joi_chain.n;
    else
        n = length(joi_types);
        joi_idxs = zeros(1,n);
        for i_idx = 1:n
            joi_idxs(i_idx) = idx_cell(joi_chain.types,joi_types{i_idx});
        end
        joi_idxs = unique(joi_idxs);
    end
    h{fig_idx,subfig_idx}.joi_idxs = joi_idxs;
    
    
    % Plot the model
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    hold on;
    
    for i_idx = h{fig_idx,subfig_idx}.joi_idxs % for each JOI
        
        % Get p and R
        p = chain.joint(joi_chain.idxs(i_idx)).p;
        R = chain.joint(joi_chain.idxs(i_idx)).R;
        ex = R(:,1); ey = R(:,2); ez = R(:,3);
        
        if PLOT_SPHERE
            if isempty(colors)
                sfc = sfc;
            else
                sfc = colors(i_idx,:);
            end
            [x,y,z] = ellipsoid(0,0,0,sr,sr,sr,30);
            fv = surf2patch(x,y,z);
            h{fig_idx,subfig_idx}.sphere{i_idx} = patch(fv,...
                'EdgeColor','none','FaceColor',sfc,'FaceAlpha',sfa);
            h{fig_idx,subfig_idx}.sphere_t{i_idx} = hgtransform;
            set(h{fig_idx,subfig_idx}.sphere{i_idx},...
                'parent',h{fig_idx,subfig_idx}.sphere_t{i_idx});
            tform = p2t(p);
            set(h{fig_idx,subfig_idx}.sphere_t{i_idx},'Matrix',tform);
        end
        
        if PRINT_JOI_NAME
            joi_name = joi_chain.types{i_idx};
            h{fig_idx,subfig_idx}.text{i_idx} = text(p(1),p(2),p(3),...
                [' ',joi_name],'fontsize',tfs,'fontname','consolas');
        end
        
        if PLOT_COORD
            h{fig_idx,subfig_idx}.cx{i_idx} = ...
                plot3([p(1),p(1)+clen*ex(1)],[p(2),p(2)+clen*ex(2)],[p(3),p(3)+clen*ex(3)],...
                'color','r','LineWidth',clw,'LineStyle','-');
            h{fig_idx,subfig_idx}.cy{i_idx} = ...
                plot3([p(1),p(1)+clen*ey(1)],[p(2),p(2)+clen*ey(2)],[p(3),p(3)+clen*ey(3)],...
                'color','g','LineWidth',clw,'LineStyle','-');
            h{fig_idx,subfig_idx}.cz{i_idx} = ...
                plot3([p(1),p(1)+clen*ez(1)],[p(2),p(2)+clen*ez(2)],[p(3),p(3)+clen*ez(3)],...
                'color','b','LineWidth',clw,'LineStyle','-');
        end
        
        if PLOT_COORD_TIP
            [Xs,Ys,Zs] = sphere(20);
            fv = surf2patch(sr_rate*clen*Xs,sr_rate*clen*Ys,sr_rate*clen*Zs);
            
            cx = p(1)+clen*ex(1);
            cy = p(2)+clen*ex(2);
            cz = p(3)+clen*ex(3);
            h{fig_idx,subfig_idx}.sx{i_idx} = patch(fv,...
                'EdgeColor','none','FaceColor','r','FaceAlpha',0.9,'facelighting','gouraud');
            h{fig_idx,subfig_idx}.sx_t{i_idx} = hgtransform;
            set(h{fig_idx,subfig_idx}.sx{i_idx},'parent',h{fig_idx,subfig_idx}.sx_t{i_idx});
            tform = p2t([cx,cy,cz]);
            set(h{fig_idx,subfig_idx}.sx_t{i_idx},'Matrix',tform);
            
            cx = p(1)+clen*ey(1);
            cy = p(2)+clen*ey(2);
            cz = p(3)+clen*ey(3);
            h{fig_idx,subfig_idx}.sy{i_idx} = patch(fv,...
                'EdgeColor','none','FaceColor','g','FaceAlpha',0.9,'facelighting','gouraud');
            h{fig_idx,subfig_idx}.sy_t{i_idx} = hgtransform;
            set(h{fig_idx,subfig_idx}.sy{i_idx},'parent',h{fig_idx,subfig_idx}.sy_t{i_idx});
            tform = p2t([cx,cy,cz]);
            set(h{fig_idx,subfig_idx}.sy_t{i_idx},'Matrix',tform);
            
            cx = p(1)+clen*ez(1);
            cy = p(2)+clen*ez(2);
            cz = p(3)+clen*ez(3);
            h{fig_idx,subfig_idx}.sz{i_idx} = patch(fv,...
                'EdgeColor','none','FaceColor','b','FaceAlpha',0.9,'facelighting','gouraud');
            h{fig_idx,subfig_idx}.sz_t{i_idx} = hgtransform;
            set(h{fig_idx,subfig_idx}.sz{i_idx},'parent',h{fig_idx,subfig_idx}.sz_t{i_idx});
            tform = p2t([cx,cy,cz]);
            set(h{fig_idx,subfig_idx}.sz_t{i_idx},'Matrix',tform);
            
        end
        
    end
    
else
    
    for i_idx = h{fig_idx,subfig_idx}.joi_idxs % for each JOI
        
        % Get p and R
        p = chain.joint(joi_chain.idxs(i_idx)).p;
        R = chain.joint(joi_chain.idxs(i_idx)).R;
        ex = R(:,1); ey = R(:,2); ez = R(:,3);
        
        if PLOT_SPHERE
            if isempty(colors)
                sfc = sfc;
            else
                sfc = colors(i_idx,:);
            end
            tform = p2t(p);
            set(h{fig_idx,subfig_idx}.sphere_t{i_idx},'Matrix',tform);
            h{fig_idx,subfig_idx}.sphere{i_idx}.FaceColor = sfc;
            h{fig_idx,subfig_idx}.sphere{i_idx}.FaceAlpha = sfa;
        end
        
        if PRINT_JOI_NAME
            joi_name = joi_chain.types{i_idx};
            h{fig_idx,subfig_idx}.text{i_idx}.Position = p;
            h{fig_idx,subfig_idx}.text{i_idx}.String = [' ',joi_name];
        end
        
        if PLOT_COORD
            h{fig_idx,subfig_idx}.cx{i_idx}.XData = [p(1),p(1)+clen*ex(1)];
            h{fig_idx,subfig_idx}.cx{i_idx}.YData = [p(2),p(2)+clen*ex(2)];
            h{fig_idx,subfig_idx}.cx{i_idx}.ZData = [p(3),p(3)+clen*ex(3)];
            
            h{fig_idx,subfig_idx}.cy{i_idx}.XData = [p(1),p(1)+clen*ey(1)];
            h{fig_idx,subfig_idx}.cy{i_idx}.YData = [p(2),p(2)+clen*ey(2)];
            h{fig_idx,subfig_idx}.cy{i_idx}.ZData = [p(3),p(3)+clen*ey(3)];
            
            h{fig_idx,subfig_idx}.cz{i_idx}.XData = [p(1),p(1)+clen*ez(1)];
            h{fig_idx,subfig_idx}.cz{i_idx}.YData = [p(2),p(2)+clen*ez(2)];
            h{fig_idx,subfig_idx}.cz{i_idx}.ZData = [p(3),p(3)+clen*ez(3)];
        end
        
        if PLOT_COORD_TIP
            cx = p(1)+clen*ex(1);
            cy = p(2)+clen*ex(2);
            cz = p(3)+clen*ex(3);
            tform = p2t([cx,cy,cz]);
            set(h{fig_idx,subfig_idx}.sx_t{i_idx},'Matrix',tform);
            
            cx = p(1)+clen*ey(1);
            cy = p(2)+clen*ey(2);
            cz = p(3)+clen*ey(3);
            tform = p2t([cx,cy,cz]);
            set(h{fig_idx,subfig_idx}.sy_t{i_idx},'Matrix',tform);
            
            cx = p(1)+clen*ez(1);
            cy = p(2)+clen*ez(2);
            cz = p(3)+clen*ez(3);
            tform = p2t([cx,cy,cz]);
            set(h{fig_idx,subfig_idx}.sz_t{i_idx},'Matrix',tform);
            
        end
        
    end
    
end