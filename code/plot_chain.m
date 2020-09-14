function fig = plot_chain(chain,varargin)
%
% Plot kinematic chain
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
if isfield(chain,'xyz_len')
    max_len = max(chain.xyz_len);
else
    max_len = 1.0;
end
def_r = max_len / 20;
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'fig_pos',[0.0,0.4,0.35,0.5]); % figure position
addParameter(p,'MULTIPLE_MONITOR',0); % handling multiple-monitors
addParameter(p,'monitor_idx',1); % index of the monitor in multiple-monitors case
addParameter(p,'view_info',[80,16]);
addParameter(p,'axis_info','');
addParameter(p,'PLOT_MESH',true);
addParameter(p,'mfc',0.5*[1,1,1]); % mesh face color
addParameter(p,'mfa',0.4); % mesh face alpha
addParameter(p,'bfc',0.5*[1,1,1]); % box face color
addParameter(p,'bfa',0.5); % box face alpha
addParameter(p,'PLOT_BCUBE',false);
addParameter(p,'PLOT_COM',false);
addParameter(p,'PLOT_LINK',true);
addParameter(p,'llc','k'); % link line color
addParameter(p,'llw',2); % link line width
addParameter(p,'lls','-'); % link line style
addParameter(p,'PLOT_ROTATE_AXIS',true);
addParameter(p,'ral',def_r); % rotate axis length
addParameter(p,'rac',''); % rotate axis color
addParameter(p,'raa',0.5); % rotate axis alpha
addParameter(p,'rasw',def_r/10); % rotate axis stem width
addParameter(p,'ratw',def_r/5); % rotate axis tip width
addParameter(p,'PLOT_JOINT_AXIS',true);
addParameter(p,'jal',def_r/2); % joint axis length
addParameter(p,'jalw',2); % joint axis line width
addParameter(p,'jals','-'); % joint axis line style
addParameter(p,'PLOT_JOINT_SPHERE',true);
addParameter(p,'jsfc','k'); % joint sphere face color
addParameter(p,'jsfa',0.5); % joint sphere face alpha
addParameter(p,'jsr',def_r/10); % joint sphere radius
addParameter(p,'PLOT_VELOCITY',false);
addParameter(p,'v_rate',1.0); % linear velocitiy arraw length rate
addParameter(p,'w_rate',0.5); % angular velocitiy arraw length rate
addParameter(p,'PRINT_JOINT_NAME',false);
addParameter(p,'LOCATE_JOINT_NAME_AT_ROTATE_AXIS',false);
addParameter(p,'jnfs',15); % joint name font size
addParameter(p,'jnfn','consolas'); % joint name font name
addParameter(p,'jnfc','k'); % joint name font color

addParameter(p,'PLOT_CAPSULE',false); % plot link capsule
addParameter(p,'cfc',0.5*[1,1,1]); % capsule face color
addParameter(p,'cfa',0.4); % capsule face alpha
addParameter(p,'cec','none'); % capsule edge color

addParameter(p,'title_str',chain.name);
addParameter(p,'tfs',20); % title font size
addParameter(p,'tfn','consolas'); % title font name

addParameter(p,'xm',''); % subaxes x margin
addParameter(p,'ym',''); % subaxes y margin


parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
fig_pos = p.Results.fig_pos;
MULTIPLE_MONITOR = p.Results.MULTIPLE_MONITOR;
monitor_idx = p.Results.monitor_idx;
view_info = p.Results.view_info;
axis_info = p.Results.axis_info;
PLOT_MESH = p.Results.PLOT_MESH;
mfc = p.Results.mfc;
mfa = p.Results.mfa;
bfc = p.Results.bfc;
bfa = p.Results.bfa;
PLOT_BCUBE = p.Results.PLOT_BCUBE;
PLOT_COM = p.Results.PLOT_COM;
PLOT_LINK = p.Results.PLOT_LINK;
llc = p.Results.llc;
llw = p.Results.llw;
lls = p.Results.lls;
PLOT_ROTATE_AXIS = p.Results.PLOT_ROTATE_AXIS;
ral = p.Results.ral;
rac = p.Results.rac;
raa = p.Results.raa;
rasw = p.Results.rasw;
ratw = p.Results.ratw;
PLOT_JOINT_AXIS = p.Results.PLOT_JOINT_AXIS;
jal = p.Results.jal;
jalw = p.Results.jalw;
jals = p.Results.jals;
PLOT_JOINT_SPHERE = p.Results.PLOT_JOINT_SPHERE;
jsr = p.Results.jsr;
jsfc = p.Results.jsfc;
jsfa = p.Results.jsfa;

PLOT_VELOCITY = p.Results.PLOT_VELOCITY;
v_rate = p.Results.v_rate;
w_rate = p.Results.w_rate;

PRINT_JOINT_NAME = p.Results.PRINT_JOINT_NAME;
LOCATE_JOINT_NAME_AT_ROTATE_AXIS = p.Results.LOCATE_JOINT_NAME_AT_ROTATE_AXIS;
jnfs = p.Results.jnfs;
jnfn = p.Results.jnfn;
jnfc = p.Results.jnfc;

PLOT_CAPSULE = p.Results.PLOT_CAPSULE;
cfc = p.Results.cfc;
cfa = p.Results.cfa;
cec = p.Results.cec;

title_str = p.Results.title_str;
tfs = p.Results.tfs;
tfn = p.Results.tfn;

xm = p.Results.xm;
ym = p.Results.ym;


% Plot start here
if h{fig_idx,subfig_idx}.first_flag || (~ishandle(h{fig_idx,subfig_idx}.fig))
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    
    % Make figure
    set_fig_position(h{fig_idx,subfig_idx}.fig,...
        'position',fig_pos,'ADD_TOOLBAR',1,'AXES_LABEL',1,...
        'view_info',view_info,'axis_info',axis_info,'SET_DRAGZOOM',1,'GRID_ON',1,...
        'MULTIPLE_MONITOR',MULTIPLE_MONITOR,'monitor_idx',monitor_idx,...
        'xm',xm,'ym',ym);
    
    % Plot mesh and/or box
    if PLOT_MESH && isfield(chain,'link') % if link exists
        for i_idx = 1:chain.n_link
            link_i = chain.link(i_idx);
            joint_idx = link_i.joint_idx;
            if (~isempty(joint_idx))
                
                % Mother joint position 
                p_link = chain.joint(joint_idx).p;
                R_link = chain.joint(joint_idx).R;
                
                % Plot mesh
                fv = link_i.fv;
                if (~isempty(fv)) % if mesh exists
                    % Plot mesh with patch
                    V = fv.vertices;
                    V = V*(1.0 + 0.01*subfig_idx);
                    h{fig_idx,subfig_idx}.mesh{i_idx} = ...
                        patch('faces',fv.faces,'vertices',V,...
                        'FaceColor', mfc, ...
                        'EdgeColor', 'none','FaceLighting','gouraud',...
                        'AmbientStrength', 0.2, 'FaceAlpha', mfa);
                    % Define hgtransform
                    h{fig_idx,subfig_idx}.mesh_t{i_idx} = hgtransform;
                    set(h{fig_idx,subfig_idx}.mesh{i_idx},...
                        'parent',h{fig_idx,subfig_idx}.mesh_t{i_idx});
                    % Move
                    tform = pr2t(p_link,R_link);
                    set(h{fig_idx,subfig_idx}.mesh_t{i_idx},'Matrix',tform);
                end
                
                % Plot box
                box_size = link_i.box;
                box_scale = link_i.box_scale;
                if (~isempty(box_size)) % if box exists
                    p_offset = link_i.p_offset;
                    R_offset = link_i.R_offset;
                    xyz_min = p_offset-box_size.*box_scale/2;
                    xyz_len = box_size.*box_scale;
                    vertex_matrix = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
                    faces_matrix = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
                    vertex_matrix = vertex_matrix.*xyz_len';
                    vertex_matrix = vertex_matrix + xyz_min'; % do basic translation
                    vertex_matrix = vertex_matrix * R_offset';
                    p_box = p_link;
                    R_box = R_link;
                    h{fig_idx,subfig_idx}.box{i_idx} = ...
                        patch('Vertices',vertex_matrix,'Faces',faces_matrix,...
                        'FaceColor',bfc,'FaceAlpha',bfa,'EdgeColor','none',...
                        'facelighting','gouraud');
                    h{fig_idx,subfig_idx}.box_t{i_idx} = hgtransform;
                    set(h{fig_idx,subfig_idx}.box{i_idx},...
                        'parent',h{fig_idx,subfig_idx}.box_t{i_idx});
                    tform = pr2t(p_box,R_box);
                    set(h{fig_idx,subfig_idx}.box_t{i_idx},'Matrix',tform);
                end
                
            end
        end
    end
    
    % Plot the bounding cube or the center of mass of a link
    if PLOT_BCUBE || PLOT_COM
        for i_idx = 1:chain.n_link 
            link_i = chain.link(i_idx);
            bcube = link_i.bcube;
            if PLOT_BCUBE && (~isempty(bcube))
                xyz_min = bcube.xyz_min;
                xyz_len = bcube.xyz_len;
                vertex_matrix = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
                faces_matrix = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
                vertex_matrix = vertex_matrix.*xyz_len';
                vertex_matrix = vertex_matrix + xyz_min'; 
                h{fig_idx,subfig_idx}.bcube{i_idx} = patch('Vertices',vertex_matrix,...
                    'Faces',faces_matrix,...
                    'FaceColor','b','FaceAlpha',0.2,...
                    'EdgeColor','k','lineWidth',1);
                h{fig_idx,subfig_idx}.bcube_t{i_idx} = hgtransform;
                set(h{fig_idx,subfig_idx}.bcube{i_idx},...
                    'parent',h{fig_idx,subfig_idx}.bcube_t{i_idx});
                tform = pr2t(chain.joint(link_i.joint_idx).p,chain.joint(link_i.joint_idx).R);
                set(h{fig_idx,subfig_idx}.bcube_t{i_idx},'Matrix',tform);
            end
            
            if PLOT_COM && (~isempty(bcube))
                sr = 0.02;
                [x,y,z] = ellipsoid(0,0,0,sr,sr,sr,30);
                fv = surf2patch(x,y,z);
                h{fig_idx,subfig_idx}.com{i_idx} = patch(fv,...
                    'EdgeColor','none','FaceColor','r','FaceAlpha',0.5,'facelighting','gouraud');
                h{fig_idx,subfig_idx}.com_t{i_idx} = hgtransform;
                set(h{fig_idx,subfig_idx}.com{i_idx},'parent',h{fig_idx,subfig_idx}.com_t{i_idx});
                T = pr2t(chain.joint(link_i.joint_idx).p,chain.joint(link_i.joint_idx).R);
                tform = T*pr2t(bcube.c_offset);
                set(h{fig_idx,subfig_idx}.com_t{i_idx},'Matrix',tform);
            end
        end
    end
    
    % Plot links
    if PLOT_LINK
        for i_idx = 1:chain.n_joint
            joint_i = chain.joint(i_idx);
            parent = joint_i.parent;
            if ~isempty(parent)
                joint_fr = chain.joint(parent);
                p1 = joint_fr.p;
                p2 = joint_i.p;
                h{fig_idx,subfig_idx}.line{i_idx} = plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],...
                    'Color',llc,'LineWidth',llw,'LineStyle',lls);
            end
        end
    end
    
    % Plot joint
    if PLOT_ROTATE_AXIS || PLOT_JOINT_AXIS || PLOT_JOINT_SPHERE || PRINT_JOINT_NAME
        for i_idx = 1:chain.n_joint % plot joints
            joint_i = chain.joint(i_idx);
            p = joint_i.p;
            R = joint_i.R;
            ex = R(:,1); ey = R(:,2); ez = R(:,3);
            
            if PLOT_ROTATE_AXIS
                a = joint_i.a; % axis
                if sum(a) ~= 0
                    axis_i = R*a;
                    p1 = p;
                    p2 = p+axis_i*ral;
                    if isempty(rac)
                        [~,max_idx] = max(a');
                        switch max_idx
                            case 1
                                rac_i = 'r';
                            case 2
                                rac_i = 'g';
                            case 3
                                rac_i = 'b';
                        end
                    else
                        rac_i = rac;
                    end
                    fv = get_arrow_3d(p1,p2,'color',rac_i,'stemWidth',rasw,'tipWidth',ratw,...
                        'facealpha',raa);
                    h{fig_idx,subfig_idx}.arrow{i_idx} = patch(fv,'facecolor',rac_i,'edgeColor','none',...
                        'FaceAlpha',raa,'FaceLighting','gouraud');
                end
            end
            if PLOT_JOINT_AXIS
                h{fig_idx,subfig_idx}.cx{i_idx} = plot3([p(1),p(1)+jal*ex(1)],[p(2),p(2)+jal*ex(2)],...
                    [p(3),p(3)+jal*ex(3)],'r','LineWidth',jalw,'LineStyle',jals);
                h{fig_idx,subfig_idx}.cy{i_idx} = plot3([p(1),p(1)+jal*ey(1)],[p(2),p(2)+jal*ey(2)],...
                    [p(3),p(3)+jal*ey(3)],'g','LineWidth',jalw,'LineStyle',jals);
                h{fig_idx,subfig_idx}.cz{i_idx} = plot3([p(1),p(1)+jal*ez(1)],[p(2),p(2)+jal*ez(2)],...
                    [p(3),p(3)+jal*ez(3)],'b','LineWidth',jalw,'LineStyle',jals);
            end
            if PLOT_JOINT_SPHERE
                [x,y,z] = ellipsoid(0,0,0,jsr,jsr,jsr,30);
                fv = surf2patch(x,y,z);
                h{fig_idx,subfig_idx}.sphere{i_idx} = patch(fv,...
                    'EdgeColor','none','FaceColor',jsfc,'FaceAlpha',jsfa,'facelighting','gouraud');
                h{fig_idx,subfig_idx}.sphere_t{i_idx} = hgtransform;
                set(h{fig_idx,subfig_idx}.sphere{i_idx},'parent',h{fig_idx,subfig_idx}.sphere_t{i_idx});
                tform = pr2t(p);
                set(h{fig_idx,subfig_idx}.sphere_t{i_idx},'Matrix',tform);
            end
            if PRINT_JOINT_NAME
                if LOCATE_JOINT_NAME_AT_ROTATE_AXIS
                    a = joint_i.a; % axis
                    axis_i = R*a;
                    text_p = p+axis_i*ral;
                else
                    text_p = p;
                end
                text_str = sprintf('[%d]%s',i_idx,joint_i.name);
                h{fig_idx,subfig_idx}.text{i_idx} = text(text_p(1),text_p(2),text_p(3),...
                    [' ',text_str],...
                    'FontSize',jnfs,'FontName',jnfn,'Color',jnfc,'Interpreter','none');
            end
        end
    end
    
    % Plot linear and angular velocities
    if PLOT_VELOCITY
        for i_idx = 1:chain.n_link
            link_i = chain.link(i_idx);
            if ~isempty(link_i.fv) % if mesh exists,
                joint_i = chain.joint(link_i.joint_idx); % joint attached to the link 
                T_joint = pr2t(joint_i.p,joint_i.R); % joint position
                T_com = T_joint*p2t(link_i.bcube.c_offset); % center of mass
                p_com = t2p(T_com);
                v = joint_i.v; % linear velocity at the world coordinate
                w = joint_i.w; % angular velocity at the world coordinate
                if h{fig_idx,subfig_idx}.first_flag
                    v = [0,0,0]';
                    w = [0,0,0]';
                end
                p_v = p_com + v*v_rate;
                p_w = p_com + w*w_rate;
                % Linear velocity (red)
                len = norm(p_com-p_v);
                fv = get_arrow_3d(p_com,p_v,'color','r','stemWidth',len/20,'tipWidth',len/10,...
                    'facealpha',0.5);
                h{fig_idx,subfig_idx}.lvel{i_idx} = patch(fv,'facecolor','r','edgeColor','none',...
                    'FaceAlpha',0.5,'FaceLighting','gouraud');
                % Angular velocity (blue)
                len = norm(p_com-p_w);
                fv = get_arrow_3d(p_com,p_w,'color','b','stemWidth',len/20,'tipWidth',len/10,...
                    'facealpha',0.5);
                h{fig_idx,subfig_idx}.avel{i_idx} = patch(fv,'facecolor','b','edgeColor','none',...
                    'FaceAlpha',0.5,'FaceLighting','gouraud');
            end
        end
    end
    
    % Plot Capsule
    if PLOT_CAPSULE
        for i_idx = 1:chain.n_link
            link_i = chain.link(i_idx);
            if ~isempty(link_i.capsule)
                cap = link_i.capsule;
                
                joint_idx = link_i.joint_idx;
                p_link = chain.joint(joint_idx).p;
                R_link = chain.joint(joint_idx).R;
                
                
                h{fig_idx,subfig_idx}.cap_patch{i_idx} = ...
                    patch('faces',cap.faces,'vertices',cap.vertices,...
                    'FaceColor', cfc, 'EdgeColor', cec,'EdgeAlpha',0.3,... % EdgeColor 'k' / 'none'
                    'FaceLighting','gouraud','AmbientStrength', 0.5, 'FaceAlpha', cfa);
                h{fig_idx,subfig_idx}.cap_patch_t{i_idx} = hgtransform;
                set(h{fig_idx,subfig_idx}.cap_patch{i_idx},...
                    'parent',h{fig_idx,subfig_idx}.cap_patch_t{i_idx});
                tform = pr2t(p_link,R_link);
                set(h{fig_idx,subfig_idx}.cap_patch_t{i_idx},'Matrix',tform);
            end
        end
    end
    
    % Plot title
    if ~isempty(title_str)
        h{fig_idx,subfig_idx}.title = title(...
            title_str,'fontsize',tfs,'fontname',tfn,'interpreter','none');
    end
    
    % Not the first anymore 
    h{fig_idx,subfig_idx}.first_flag = false;
else
    
    % Plot mesh
    if PLOT_MESH && isfield(chain,'link') % only if link exists
        for i_idx = 1:chain.n_link
            link_i = chain.link(i_idx);
            fv = link_i.fv;
            joint_idx = link_i.joint_idx;
            if (~isempty(joint_idx))
                p_link = chain.joint(joint_idx).p;
                R_link = chain.joint(joint_idx).R;
                
                % Plot mesh
                if ~isempty(fv) % if mesh exists
                    % Move
                    tform = pr2t(p_link,R_link);
                    set(h{fig_idx,subfig_idx}.mesh_t{i_idx},'Matrix',tform);
                end
                
                % Plot box
                box_size = link_i.box;
                if (~isempty(box_size)) % if box exists
                    p_box = p_link;
                    R_box = R_link;
                    tform = pr2t(p_box,R_box);
                    set(h{fig_idx,subfig_idx}.box_t{i_idx},'Matrix',tform);
                end
                
            end
        end
    end
    
    % Plot the bounding cube or the center of mass of a link
    if PLOT_BCUBE || PLOT_COM
        for i_idx = 1:chain.n_link 
            link_i = chain.link(i_idx);
            bcube = link_i.bcube;
            if PLOT_BCUBE && (~isempty(bcube))
                tform = pr2t(chain.joint(link_i.joint_idx).p,chain.joint(link_i.joint_idx).R);
                set(h{fig_idx,subfig_idx}.bcube_t{i_idx},'Matrix',tform);
            end
            
            if PLOT_COM && (~isempty(bcube))
                T = pr2t(chain.joint(link_i.joint_idx).p,chain.joint(link_i.joint_idx).R);
                tform = T*pr2t(bcube.c_offset);
                set(h{fig_idx,subfig_idx}.com_t{i_idx},'Matrix',tform);
            end
        end
    end
    
    % Plot links
    if PLOT_LINK
        for i_idx = 1:chain.n_joint
            joint_i = chain.joint(i_idx);
            parent = joint_i.parent;
            if ~isempty(parent)
                joint_fr = chain.joint(parent);
                p1 = joint_fr.p;
                p2 = joint_i.p;
                h{fig_idx,subfig_idx}.line{i_idx}.XData = [p1(1),p2(1)];
                h{fig_idx,subfig_idx}.line{i_idx}.YData = [p1(2),p2(2)];
                h{fig_idx,subfig_idx}.line{i_idx}.ZData = [p1(3),p2(3)];
            end
        end
    end
    
    % Plot joint
    if PLOT_ROTATE_AXIS || PLOT_JOINT_AXIS || PLOT_JOINT_SPHERE || PRINT_JOINT_NAME
        for i_idx = 1:chain.n_joint % plot joints
            joint_i = chain.joint(i_idx);
            p = joint_i.p;
            R = joint_i.R;
            ex = R(:,1); ey = R(:,2); ez = R(:,3);
            if PLOT_ROTATE_AXIS
                a = joint_i.a; % axis
                if sum(a) ~= 0
                    axis_i = R*a;
                    p1 = p;
                    p2 = p+axis_i*ral;
                    fv = get_arrow_3d(p1,p2,'color','b','stemWidth',rasw,'tipWidth',ratw,...
                        'facealpha',raa);
                    h{fig_idx,subfig_idx}.arrow{i_idx}.Faces = fv.faces;
                    h{fig_idx,subfig_idx}.arrow{i_idx}.Vertices = fv.vertices;
                end
            end
            if PLOT_JOINT_AXIS
                h{fig_idx,subfig_idx}.cx{i_idx}.XData = [p(1),p(1)+jal*ex(1)];
                h{fig_idx,subfig_idx}.cx{i_idx}.YData = [p(2),p(2)+jal*ex(2)];
                h{fig_idx,subfig_idx}.cx{i_idx}.ZData = [p(3),p(3)+jal*ex(3)];
                h{fig_idx,subfig_idx}.cy{i_idx}.XData = [p(1),p(1)+jal*ey(1)];
                h{fig_idx,subfig_idx}.cy{i_idx}.YData = [p(2),p(2)+jal*ey(2)];
                h{fig_idx,subfig_idx}.cy{i_idx}.ZData = [p(3),p(3)+jal*ey(3)];
                h{fig_idx,subfig_idx}.cz{i_idx}.XData = [p(1),p(1)+jal*ez(1)];
                h{fig_idx,subfig_idx}.cz{i_idx}.YData = [p(2),p(2)+jal*ez(2)];
                h{fig_idx,subfig_idx}.cz{i_idx}.ZData = [p(3),p(3)+jal*ez(3)];
            end
            if PLOT_JOINT_SPHERE
                tform = pr2t(p);
                set(h{fig_idx,subfig_idx}.sphere_t{i_idx},'Matrix',tform);
            end
            if PRINT_JOINT_NAME
                if LOCATE_JOINT_NAME_AT_ROTATE_AXIS
                    a = joint_i.a; % axis
                    axis_i = R*a;
                    text_p = p+axis_i*ral;
                else
                    text_p = p;
                end
                text_str = sprintf('[%d]%s',i_idx,joint_i.name);
                h{fig_idx,subfig_idx}.text{i_idx}.Position = text_p;
                h{fig_idx,subfig_idx}.text{i_idx}.String = text_str;
            end
        end
    end
    
    
    % Plot linear and angular velocities
    if PLOT_VELOCITY
        for i_idx = 1:chain.n_link
            link_i = chain.link(i_idx);
            if ~isempty(link_i.fv) % if mesh exists,
                joint_i = chain.joint(link_i.joint_idx);
                T_joint = pr2t(joint_i.p,joint_i.R); % actual link position
                T_com = T_joint*p2t(link_i.bcube.c_offset); % center of mass
                p_com = t2p(T_com);
                v = joint_i.v; % linear velocity at the world coordinate
                w = joint_i.w; % angular velocity at the world coordinate
                p_v = p_com + v*v_rate;
                p_w = p_com + w*w_rate;
                % Linear velocity
                len = norm(p_com-p_v);
                fv = get_arrow_3d(p_com,p_v,'color','r','stemWidth',len/20,'tipWidth',len/10,...
                    'facealpha',0.5);
                h{fig_idx,subfig_idx}.lvel{i_idx}.Faces = fv.faces;
                h{fig_idx,subfig_idx}.lvel{i_idx}.Vertices = fv.vertices;
                % Angular velocity
                len = norm(p_com-p_w);
                fv = get_arrow_3d(p_com,p_w,'color','b','stemWidth',len/20,'tipWidth',len/10,...
                    'facealpha',0.5);
                h{fig_idx,subfig_idx}.avel{i_idx}.Faces = fv.faces;
                h{fig_idx,subfig_idx}.avel{i_idx}.Vertices = fv.vertices;
            end
        end
    end
    
    % Plot Capsule
    if PLOT_CAPSULE
        for i_idx = 1:chain.n_link
            link_i = chain.link(i_idx);
            if ~isempty(link_i.capsule)
                
                joint_idx = link_i.joint_idx;
                p_link = chain.joint(joint_idx).p;
                R_link = chain.joint(joint_idx).R;

                tform = pr2t(p_link,R_link);
                set(h{fig_idx,subfig_idx}.cap_patch_t{i_idx},'Matrix',tform);
            end
        end
    end
    
    
    % Plot title
    if ~isempty(title_str)
        h{fig_idx,subfig_idx}.title.String = title_str;
    end
end

fig = h{fig_idx,subfig_idx}.fig;
