function plot_spheres(ps,varargin)
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
addParameter(p,'sr',0.1); % sphere radius
addParameter(p,'sfc',''); % sphere face color
addParameter(p,'sfa',0.5); % sphere face alpha
addParameter(p,'colors',''); % colors
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
sr = p.Results.sr;
sfc = p.Results.sfc;
sfa = p.Results.sfa;
colors = p.Results.colors;

n = size(ps,1);
if isempty(colors)
    colors = linspecer(n);
end

% Plot start here
if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    hold on;
    
    for i_idx = 1:n
        p = ps(i_idx,:);
        [x,y,z] = ellipsoid(0,0,0,sr,sr,sr,30);
        fv = surf2patch(x,y,z);
        if isempty(sfc)
            sfc_i = colors(i_idx,:);
        else
            sfc_i = sfc;
        end
        h{fig_idx,subfig_idx}.sphere{i_idx} = patch(fv,...
            'EdgeColor','none','FaceColor',sfc_i,'FaceAlpha',sfa,'facelighting','gouraud');
        h{fig_idx,subfig_idx}.sphere_t{i_idx} = hgtransform;
        set(h{fig_idx,subfig_idx}.sphere{i_idx},...
            'parent',h{fig_idx,subfig_idx}.sphere_t{i_idx});
        tform = pr2t(p);
        set(h{fig_idx,subfig_idx}.sphere_t{i_idx},'Matrix',tform);
    end
    
else
    for i_idx = 1:n
        p = ps(i_idx,:);        
        tform = pr2t(p);
        set(h{fig_idx,subfig_idx}.sphere_t{i_idx},'Matrix',tform);
    end
end


