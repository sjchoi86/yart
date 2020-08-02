function plot_ik_targets(chain_model,ik,varargin)
%
% Plot IK targets
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
chain_sz = get_chain_sz(chain_model);
def_r = max(chain_sz.xyz_len)/30;
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'PLOT_SPHERE_TRGT',1); % plot sphere of target joints
addParameter(p,'PLOT_SPHERE_MODEL',1); % plot sphere of model joints
addParameter(p,'sr',def_r); % sphere radius
addParameter(p,'sfc',''); % sphere face color
addParameter(p,'sfa',0.5); % sphere face alpha
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
PLOT_SPHERE_TRGT = p.Results.PLOT_SPHERE_TRGT;
PLOT_SPHERE_MODEL = p.Results.PLOT_SPHERE_MODEL;
sr = p.Results.sr;
sfc = p.Results.sfc;
sfa = p.Results.sfa;

% Plot start here
if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    colors = linspecer(ik.n_target);
    for i_idx = 1:ik.n_target
        ik_target = ik.targets(i_idx);
        p_trgt = ik_target.p;
        R_trgt = ik_target.R;
        temp_idx = idx_cell(chain_model.joint_names,ik_target.joint_name);
        p_model = chain_model.joint(temp_idx).p;
        if isempty(sfc)
            sfc_i = colors(i_idx,:);
        else
            sfc_i = sfc;
        end
        
        if PLOT_SPHERE_TRGT
            [x,y,z] = ellipsoid(0,0,0,sr,sr,sr,30);
            fv = surf2patch(x,y,z);
            h{fig_idx,subfig_idx}.sphere_trgt{i_idx} = patch(fv,...
                'EdgeColor','none','FaceColor',sfc_i,'FaceAlpha',sfa,'facelighting','gouraud');
            h{fig_idx,subfig_idx}.sphere_t_trgt{i_idx} = hgtransform;
            set(h{fig_idx,subfig_idx}.sphere_trgt{i_idx},...
                'parent',h{fig_idx,subfig_idx}.sphere_t_trgt{i_idx});
            tform = pr2t(p_trgt);
            set(h{fig_idx,subfig_idx}.sphere_t_trgt{i_idx},'Matrix',tform);
        end
        
        if PLOT_SPHERE_MODEL
            [x,y,z] = ellipsoid(0,0,0,sr,sr,sr,30);
            fv = surf2patch(x,y,z);
            h{fig_idx,subfig_idx}.sphere_model{i_idx} = patch(fv,...
                'EdgeColor','none','FaceColor',sfc_i,'FaceAlpha',sfa,'facelighting','gouraud');
            h{fig_idx,subfig_idx}.sphere_t_model{i_idx} = hgtransform;
            set(h{fig_idx,subfig_idx}.sphere_model{i_idx},...
                'parent',h{fig_idx,subfig_idx}.sphere_t_model{i_idx});
            tform = pr2t(p_model);
            set(h{fig_idx,subfig_idx}.sphere_t_model{i_idx},'Matrix',tform);
        end
    end
    
else
    
    for i_idx = 1:ik.n_target
        ik_target = ik.targets(i_idx);
        p_trgt = ik_target.p;
        R_trgt = ik_target.R;
        temp_idx = idx_cell(chain_model.joint_names,ik_target.joint_name);
        p_model = chain_model.joint(temp_idx).p;
        
        if PLOT_SPHERE_TRGT
            tform = pr2t(p_trgt);
            set(h{fig_idx,subfig_idx}.sphere_t_trgt{i_idx},'Matrix',tform);
        end
        
        if PLOT_SPHERE_MODEL
            tform = pr2t(p_model);
            set(h{fig_idx,subfig_idx}.sphere_t_model{i_idx},'Matrix',tform);
        end
        
    end
    
end