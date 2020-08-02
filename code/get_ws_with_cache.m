function ws = get_ws_with_cache(chain_model,joi_model,varargin)
%
% Get the workspaces of the robot model
%

% Parse input options
p = inputParser;
addParameter(p,'ws_joi_types',{'rh','re','lh','le'});
addParameter(p,'RE',false);
addParameter(p,'cache_folder','../cache'); % folder to caache
addParameter(p,'PLOT_EACH',0);
addParameter(p,'PLOT_FINAL',0);
parse(p,varargin{:});
ws_joi_types = p.Results.ws_joi_types;
RE = p.Results.RE;
cache_folder = p.Results.cache_folder;
PLOT_EACH = p.Results.PLOT_EACH;
PLOT_FINAL = p.Results.PLOT_FINAL;


% Cache file
cache_path = sprintf('%s/model/ws_%s.mat',cache_folder,chain_model.name);
if exist(cache_path,'file') && (RE==0) % if cache exists, load
    l = load(cache_path);
    ws = l.ws;
else
    % Workspace
    ws_info = cell(1,length(ws_joi_types));
    dir_types = {'up','down','left','right','forward','backward'};
    colors = linspecer(length(ws_info));
    
    for ws_idx = 1:length(ws_joi_types) % for different workspaces
        for dir_idx = 1:length(dir_types) % for different directions
            
            % Configuration
            joi_type = ws_joi_types{ws_idx};
            ws_col = colors(ws_idx,:); % workspace color
            dir_type = dir_types{dir_idx};
            margin = 2;
            switch dir_type
                case 'up', p_offset = [0,0,margin]';
                case 'down', p_offset = [0,0,-margin]';
                case 'left', p_offset = [0,margin,0]';
                case 'right', p_offset = [0,-margin,0]';
                case 'forward', p_offset = [margin,0,0]';
                case 'backward', p_offset = [-margin,0,0]';
            end
            
            % Set IK
            chain_model = update_chain_q(...
                chain_model,chain_model.rev_joint_names,zeros(1,chain_model.n_rev_joint));
            chain_model = fk_chain(chain_model);
            joi_idx = get_joint_idx_from_joi_type(joi_model,joi_type);
            joi_name = get_joint_name_from_joi_type(chain_model,joi_model,joi_type);
            p_trgt = chain_model.joint(joi_idx).p + p_offset;
            ik = init_ik(chain_model); % init IK
            ik = add_ik(ik,'joint_name',joi_name,'p',p_trgt,'IK_P',1); % add IK target
            
            % Loop for IK
            q = get_q_chain(chain_model,ik.joint_names_control);
            while 1
                % Run IK
                [ik,chain_model,q,~] = onestep_ik(ik,chain_model,q);
                p_curr = chain_model.joint(joi_idx).p;
                ws_info{ws_idx} = get_minmax(ws_info{ws_idx},p_curr);
                
                % Plot
                if PLOT_EACH
                    plot_chain(chain_model,...
                        'fig_idx',1,'subfig_idx',1,'view_info',[88,4],'axis_info','',...
                        'PLOT_MESH',1,'mfa',0.2,'PLOT_LINK',1,'PLOT_ROTATE_AXIS',1,'PLOT_JOINT_AXIS',0,...
                        'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0,'PLOT_CAPSULE',0,...
                        'title_str',sprintf('[%s]-[%s]-[%s] tick:[%d] tick_dec:[%d]',...
                        chain_model.name,joi_type,dir_type,ik.tick,ik.tick_dec));
                    plot_joi_chain(chain_model,joi_model,'joi_types','','PLOT_COORD',0,...
                        'PLOT_SPHERE',1,'sr',0.02,'sfa',0.6,'colors',linspecer(joi_model.n));
                    plot_ik_targets(chain_model,ik,'PLOT_SPHERE_TRGT',1,'sfc',ws_col);
                    for i_idx = 1:length(ws_info)
                        ws_i = ws_info{i_idx};
                        if isempty(ws_i), continue; end
                        plot_cube(p2t([0,0,0]),ws_i.min_vals,ws_i.len_vals,...
                            'fig_idx',1,'subfig_idx',i_idx,'color',ws_col,'alpha',0.2,'edge_color','k');
                    end
                    drawnow limitrate;
                end
                
                % Terminate
                if ik.tick_dec >= 10
                    break;
                end
            end
            
        end % for dir_idx = 1:length(dir_types) % for different directions
    end % for ws_idx = 1:length(ws_joi_types) % for different workspaces
    if PLOT_EACH
        ca; % close all
    end
    
    % Workspace Result
    ws.joi_types = ws_joi_types;
    ws.info = ws_info;
    ws.n = length(ws_joi_types);
    
    % Save
    [p,~,~] = fileparts(cache_path);
    make_dir_if_not_exist(p);
    save(cache_path,'ws');
    fprintf(2,'[%s] saved.\n',cache_path);
    
end


% Plot final computed workspaces of JOI
if PLOT_FINAL 
    chain_model = update_chain_q(...
        chain_model,chain_model.rev_joint_names,zeros(1,chain_model.n_rev_joint));
    chain_model = fk_chain(chain_model);
    plot_chain(chain_model,...
        'fig_idx',1,'subfig_idx',1,'view_info',[88,4],'axis_info','',...
        'PLOT_MESH',1,'mfa',0.2,'PLOT_LINK',1,'PLOT_ROTATE_AXIS',1,'PLOT_JOINT_AXIS',0,...
        'PLOT_JOINT_SPHERE',0,'PRINT_JOINT_NAME',0,'PLOT_CAPSULE',0,...
        'title_str',sprintf('[%s]',chain_model.name));
    plot_joi_chain(chain_model,joi_model,'joi_types','','PLOT_COORD',0,...
        'PLOT_SPHERE',1,'sr',0.02,'sfa',0.6,'colors',linspecer(joi_model.n));
    colors = linspecer(length(ws.info));
    for i_idx = 1:length(ws.info)
        ws_col = colors(i_idx,:);
        ws_i = ws.info{i_idx};
        hs(i_idx) = plot_cube(p2t([0,0,0]),ws_i.min_vals,ws_i.len_vals,...
            'fig_idx',1,'subfig_idx',i_idx,'color',ws_col,'alpha',0.2,'edge_color','k');
        strs{i_idx} = ws.joi_types{i_idx};
    end
    legend(hs,strs,'fontsize',20,'fontname','consolas','Location','SouthEast');
end

