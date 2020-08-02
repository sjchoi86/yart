function plot_cubes(Ts,xyz_mins,xyz_lens,varargin)
%
% Plot multiple cubes
%
persistent h

% Make enough handlers at the first
if isempty(h), for i = 1:10, for j = 1:100, h{i,j}.first_flag = true; end; end; end

% Parse options
p = inputParser;
addParameter(p,'fig_idx',1);
addParameter(p,'subfig_idx',1);
addParameter(p,'color','');
addParameter(p,'colors','');
addParameter(p,'alpha',0.2);
addParameter(p,'edge_color','none');
addParameter(p,'lw',1);
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
subfig_idx = p.Results.subfig_idx;
color = p.Results.color;
colors = p.Results.colors;
alpha = p.Results.alpha;
edge_color = p.Results.edge_color;
lw = p.Results.lw;

% Number of cubes
n = length(Ts);

if h{fig_idx,subfig_idx}.first_flag || ~ishandle(h{fig_idx,subfig_idx}.fig)
    h{fig_idx,subfig_idx}.first_flag = false;
    h{fig_idx,subfig_idx}.fig = figure(fig_idx);
    
    % Plot cubes
    for i_idx = 1:n
        T = Ts{i_idx};
        xyz_min = xyz_mins{i_idx};
        xyz_len = xyz_lens{i_idx};
        % Cube configuration
        xyz_min = reshape(xyz_min,[1,3]);
        xyz_len = reshape(xyz_len,[1,3]);
        [p,R] = t2pr(T);
        vertex_matrix = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
        faces_matrix = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
        vertex_matrix = vertex_matrix.*xyz_len;
        vertex_matrix = vertex_matrix + xyz_min; % do basic translation
        
        if ~isempty(color)
            color_i = color;
        elseif ~isempty(colors)
            color_i = colors(i_idx,:);
        else
            color_i = 0.5*[1,1,1];
        end
        
        h{fig_idx,subfig_idx}.p{i_idx} = p;
        
        h{fig_idx,subfig_idx}.vertex_matrix{i_idx} = vertex_matrix;
        h{fig_idx,subfig_idx}.patch{i_idx} = patch('Vertices',vertex_matrix,...
            'Faces',faces_matrix,...
            'FaceColor',color_i,'FaceAlpha',alpha,...
            'EdgeColor',edge_color,'lineWidth',lw); % 'EdgeColor','none');
        h{fig_idx,subfig_idx}.patch_t{i_idx} = hgtransform;
        set(h{fig_idx,subfig_idx}.patch{i_idx},'parent',h{fig_idx,subfig_idx}.patch_t{i_idx});
        tform = pr2t(p,R);
        set(h{fig_idx,subfig_idx}.patch_t{i_idx},'Matrix',tform);
    end
    
else
    
    % Plot cubes
    for i_idx = 1:n
        T = Ts{i_idx};
        xyz_min = xyz_mins{i_idx};
        xyz_len = xyz_lens{i_idx};
        % Cube configuration
        xyz_min = reshape(xyz_min,[1,3]);
        xyz_len = reshape(xyz_len,[1,3]);
        [p,R] = t2pr(T);
        vertex_matrix = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
        faces_matrix = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
        vertex_matrix = vertex_matrix.*xyz_len;
        vertex_matrix = vertex_matrix + xyz_min; % do basic translation
        
        if ~isempty(color)
            color_i = color;
        elseif ~isempty(colors)
            color_i = colors(i_idx,:);
        else
            color_i = 0.5*[1,1,1];
        end
        
        if ~isequal(h{fig_idx,subfig_idx}.vertex_matrix{i_idx},vertex_matrix)
            h{fig_idx,subfig_idx}.vertex_matrix{i_idx} = vertex_matrix;
            delete(h{fig_idx,subfig_idx}.patch{i_idx});
            h{fig_idx,subfig_idx}.patch{i_idx} = patch('Vertices',vertex_matrix,...
                'Faces',faces_matrix,...
                'FaceColor',color_i,'FaceAlpha',alpha,...
                'EdgeColor',edge_color,'lineWidth',lw); % 'EdgeColor','none');
            h{fig_idx,subfig_idx}.patch_t{i_idx} = hgtransform;
            set(h{fig_idx,subfig_idx}.patch{i_idx},'parent',h{fig_idx,subfig_idx}.patch_t{i_idx});
        end
        if (~isequal(h{fig_idx,subfig_idx}.vertex_matrix{i_idx},vertex_matrix)) || ...
                (~isequal(h{fig_idx,subfig_idx}.p{i_idx},p))
            tform = pr2t(p,R);
            set(h{fig_idx,subfig_idx}.patch_t{i_idx},'Matrix',tform);
            h{fig_idx,subfig_idx}.p{i_idx} = p;
        end
    end
    
end
