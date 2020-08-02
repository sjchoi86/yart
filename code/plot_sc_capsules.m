function plot_sc_capsules(chain_model,sc_ij_list,varargin)
%
% Plot self-collided capsules
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'cfa',0.4);
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
cfa = p.Results.cfa;

% Plot start here
if h{fig_idx,subfig_idx}.first_flag || (~ishandle(h{fig_idx,subfig_idx}.fig))
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    h{fig_idx,subfig_idx}.first_flag = false;
    
    n_sc = size(sc_ij_list,1);
    for sc_idx = 1:n_sc % for self-collided pairs
        sc_ij = sc_ij_list(sc_idx,:); sc_i = sc_ij(1); sc_j = sc_ij(2);
        
        joint_idx_i = chain_model.link(sc_i).joint_idx;
        joint_idx_j = chain_model.link(sc_j).joint_idx;
        p_link_i = chain_model.joint(joint_idx_i).p;
        R_link_i = chain_model.joint(joint_idx_i).R;
        p_link_j = chain_model.joint(joint_idx_j).p;
        R_link_j = chain_model.joint(joint_idx_j).R;
        cap_i = chain_model.link(sc_i).capsule;
        cap_j = chain_model.link(sc_j).capsule;
        
        cap_i2 = get_capsule_shape(cap_i.p,cap_i.R,cap_i.radius*1.05,cap_i.height*1.05);
        cap_j2 = get_capsule_shape(cap_j.p,cap_j.R,cap_j.radius*1.05,cap_j.height*1.05);
        
        h{fig_idx,subfig_idx}.cap_patch_i{sc_idx} = ...
            patch('faces',cap_i2.faces,'vertices',transform_vertex(cap_i2.vertices,p_link_i,R_link_i),...
            'FaceColor','r', 'EdgeColor','k','EdgeAlpha',0.3,...
            'FaceLighting','gouraud','AmbientStrength', 0.5, 'FaceAlpha', cfa);
        h{fig_idx,subfig_idx}.cap_patch_j{sc_idx} = ...
            patch('faces',cap_j2.faces,'vertices',transform_vertex(cap_j2.vertices,p_link_j,R_link_j),...
            'FaceColor','r', 'EdgeColor','k','EdgeAlpha',0.3,...
            'FaceLighting','gouraud','AmbientStrength', 0.5, 'FaceAlpha', cfa);
    end
    
    if n_sc == 0
        h{fig_idx,subfig_idx}.cap_patch_i = '';
        h{fig_idx,subfig_idx}.cap_patch_j = '';
    end
    
else
    
    if ~isempty(h{fig_idx,subfig_idx}.cap_patch_i) || ...
            ~isempty(h{fig_idx,subfig_idx}.cap_patch_j)
        n_sc = length(h{fig_idx,subfig_idx}.cap_patch_i);
        for sc_idx = 1:n_sc
            delete(h{fig_idx,subfig_idx}.cap_patch_i{sc_idx});
            delete(h{fig_idx,subfig_idx}.cap_patch_j{sc_idx});
        end
    end
    
    n_sc = size(sc_ij_list,1);
    for sc_idx = 1:n_sc % for self-collided pairs
        sc_ij = sc_ij_list(sc_idx,:); sc_i = sc_ij(1); sc_j = sc_ij(2);
        
        joint_idx_i = chain_model.link(sc_i).joint_idx;
        joint_idx_j = chain_model.link(sc_j).joint_idx;
        p_link_i = chain_model.joint(joint_idx_i).p;
        R_link_i = chain_model.joint(joint_idx_i).R;
        p_link_j = chain_model.joint(joint_idx_j).p;
        R_link_j = chain_model.joint(joint_idx_j).R;
        cap_i = chain_model.link(sc_i).capsule;
        cap_j = chain_model.link(sc_j).capsule;
        
        cap_i2 = get_capsule_shape(cap_i.p,cap_i.R,cap_i.radius*1.05,cap_i.height*1.05);
        cap_j2 = get_capsule_shape(cap_j.p,cap_j.R,cap_j.radius*1.05,cap_j.height*1.05);
        
        h{fig_idx,subfig_idx}.cap_patch_i{sc_idx} = ...
            patch('faces',cap_i2.faces,'vertices',transform_vertex(cap_i2.vertices,p_link_i,R_link_i),...
            'FaceColor','r', 'EdgeColor','k','EdgeAlpha',0.3,...
            'FaceLighting','gouraud','AmbientStrength', 0.5, 'FaceAlpha', cfa);
        h{fig_idx,subfig_idx}.cap_patch_j{sc_idx} = ...
            patch('faces',cap_j2.faces,'vertices',transform_vertex(cap_j2.vertices,p_link_j,R_link_j),...
            'FaceColor','r', 'EdgeColor','k','EdgeAlpha',0.3,...
            'FaceLighting','gouraud','AmbientStrength', 0.5, 'FaceAlpha', cfa);
    end
    
    if n_sc == 0
        h{fig_idx,subfig_idx}.cap_patch_i = '';
        h{fig_idx,subfig_idx}.cap_patch_j = '';
    end
    
end

