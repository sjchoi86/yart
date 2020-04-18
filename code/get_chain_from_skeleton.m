function chains = get_chain_from_skeleton(skeleton,time,varargin)
%
% Get the kinematic chain
%

% Parse input options
ps = inputParser;
ps.addParameter('USE_METER',1); % use a meter convention
ps.addParameter('ROOT_AT_ORIGIN',1); % place the root at the origin
ps.addParameter('Z_UP',1); % z-axis up
ps.addParameter('HZ',30); % frequency
parse(ps,varargin{:});
USE_METER = ps.Results.USE_METER;
ROOT_AT_ORIGIN = ps.Results.ROOT_AT_ORIGIN;
Z_UP = ps.Results.Z_UP;
HZ = ps.Results.HZ;

ticks = round(linspace(1,length(time),round((time(end)-time(1))*HZ)));
L = length(ticks);
chains = cell(1,L);
cnt = 0;
for tick = ticks
    % Get kinematic chain
    chain = [];
    chain.name = '';
    for i_idx = 1:length(skeleton)
        s = skeleton(i_idx);
        name = s.name;
        if ~isempty(s.trans)
            T = s.trans(:,:,tick);
            [p,R] = t2pr(T);
        else
            p = [0,0,0]';
            R = eye(3,3);
        end
        T_offset = s.toffsets(:,:,tick);
        [p_offset,R_offset] = t2pr(T_offset);
        parent = s.parent;
        if parent == 0
            parent = [];
        end
        chain.joint(i_idx).name = name;
        chain.joint(i_idx).p = p;
        chain.joint(i_idx).R = R;
        TO_METER = 0.056444;
        if USE_METER
            chain.joint(i_idx).p_offset = p_offset*TO_METER; % resize to a meter scale
        else
            chain.joint(i_idx).p_offset = p_offset;
        end
        chain.joint(i_idx).R_offset = R_offset;
        chain.joint(i_idx).parent = parent;
    end
    chain.n_joint = length(chain.joint);
    % Get the child structure
    parents = cell(1,length(skeleton));
    for i_idx = 1:length(skeleton)
        s = skeleton(i_idx);
        parents{i_idx} = s.parent;
    end
    childs = get_childs_from_parents(parents);
    % Add the child structure
    for i_idx = 1:length(skeleton)
        chain.joint(i_idx).childs = childs{i_idx};
    end
    
    % FK to make sure
    if ROOT_AT_ORIGIN
        chain.joint(get_topmost_idx(chain)).p = [0,0,0]';
    end
    if Z_UP
        chain.joint(get_topmost_idx(chain)).R = rpy2r([0,0,pi/2]')*rpy2r([pi/2,0,0]')*...
            chain.joint(get_topmost_idx(chain)).R;
    end
    chain = fk_chain(chain);
    
    % Append chain
    cnt = cnt + 1;
    chains{cnt} = chain;
end
