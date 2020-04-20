function fig = plot_chain_graph(chain,varargin)

% Parse input options
p = inputParser;
addParameter(p,'fig_idx',2);
addParameter(p,'position',[0.0,0.0,0.5,0.3]);
addParameter(p,'text_fs',15);
addParameter(p,'title_str',chain.name);
addParameter(p,'title_fs',25);
parse(p,varargin{:});
fig_idx = p.Results.fig_idx;
position = p.Results.position;
text_fs = p.Results.text_fs;
title_str = p.Results.title_str;
title_fs = p.Results.title_fs;

n = chain.n_joint; % number of joint
parents = zeros(1,n);
for i_idx = 1:n
    if ~isempty(chain.joint(i_idx).parent)
        parents(i_idx) = chain.joint(i_idx).parent;
    end
end
[x,y,~]=treelayout(parents); % get tree structure
f = find(parents~=0);
pp = parents(f);
X = [x(f); x(pp); NaN(size(f))];
Y = [y(f); y(pp); NaN(size(f))];
X = X(:);
Y = Y(:);

% Set figure
fig = set_fig_position(figure(fig_idx),'position',position,...
    'ADD_TOOLBAR',1,'view_info','',...
    'AXIS_EQUAL',0,'AXES_LABEL',0,'GRID_ON',0);
axis off; subaxes(fig,1,1,1,0.05,0.15); axis off; hold on;
x_offset = 0.0;
plot (X+x_offset, Y, 'k-','MarkerSize',15,'LineWidth',2,'MarkerFaceColor','w');
for i_idx = 1:n % for all joints
    % Plot node
    if isfield(chain.joint(i_idx),'a') % if rotational axis exists
        if sum(chain.joint(i_idx).a) == 0
            color = 'w';
        else
            [~,max_idx] = max(abs(chain.joint(i_idx).a'));
            rgb = {'r','g','b'};
            color = rgb{max_idx};
        end
    else
        color = 'w';
    end
    plot (x(i_idx)+x_offset,y(i_idx),'ko',...
        'MarkerSize',14,'LineWidth',2,'MarkerFaceColor',...
        color);
    str = sprintf(' [%d]%s',i_idx,chain.joint(i_idx).name);
    % Text joint name
    text(x(i_idx)+x_offset,y(i_idx),str,'FontSize',text_fs,...
        'Interpreter','none','FontName','Consolas');
end

% Title
title(title_str,'fontsize',title_fs,'fontname','Consolas');

x_margin = 0.05;
xlim([0,1+x_margin]);
set(gcf,'Color','w');
axis off;
dragzoom;
