function chain = get_chain_from_urdf(model_name,urdf_path,varargin)
%
% Here, we follow the parameters defined by Kajita et al. except we use multiple childs structure.
% We separate joints from links by adapting this multiple childs structure.
%

% Parse options
p = inputParser;
addParameter(p,'CENTER_ROBOT',false); % redo loading
parse(p,varargin{:});
CENTER_ROBOT = p.Results.CENTER_ROBOT;

% Parse URDF
s = xml2struct(urdf_path);
robot = s.robot;

% Parse joint information
chain = struct();
chain.name = model_name;
chain.robot = robot;
chain.n_joint = length(robot.joint); % number of joint
chain.n_rev_joint = 0;
chain.joint_names = cell(1,chain.n_joint);
chain.rev_joint_names = [];
chain.parent_link_names = cell(1,chain.n_joint); % temporary parent link names
chain.child_link_names = cell(1,chain.n_joint); % temporary child link names
for i_idx = 1:chain.n_joint
    joint_i = robot.joint{i_idx}; % joint
    chain.joint(i_idx).name = joint_i.Attributes.name; % joint name
    chain.joint_names{i_idx} = chain.joint(i_idx).name;
    chain.joint(i_idx).parent = [];
    chain.joint(i_idx).childs = [];
    chain.joint(i_idx).p = [0,0,0]';
    chain.joint(i_idx).R = eye(3,3);
    chain.joint(i_idx).q = 0;
    chain.joint(i_idx).q_prev = 0;
    chain.joint(i_idx).q_diff = 0;
    chain.joint(i_idx).type = joint_i.Attributes.type; % joint type (revolute/fixed)
    % Joint revolute axis
    if isequal(chain.joint(i_idx).type,'revolute')
        chain.joint(i_idx).a = str2num(joint_i.axis.Attributes.xyz)'; % revolute axis
        chain.n_rev_joint = chain.n_rev_joint + 1;
        chain.rev_joint_names{chain.n_rev_joint} = chain.joint(i_idx).name; % append revolute joint name
    else
        chain.joint(i_idx).a = [0,0,0]';
    end
    % Joint limits
    try
        limit = str2num([joint_i.limit.Attributes.lower,',',joint_i.limit.Attributes.upper]);
    catch
        limit = [0,0];
    end
    chain.joint(i_idx).limit = limit;
    % Joint offset
    chain.joint(i_idx).p_offset = str2num(joint_i.origin.Attributes.xyz)';
    chain.joint(i_idx).R_offset = rpy2r(str2num(joint_i.origin.Attributes.rpy));
    % Parent and child links
    chain.joint(i_idx).parent_link = joint_i.parent.Attributes.link;
    chain.joint(i_idx).child_link = joint_i.child.Attributes.link;
    chain.parent_link_names{i_idx} = joint_i.parent.Attributes.link;
    chain.child_link_names{i_idx} = joint_i.child.Attributes.link;
    % Velocity
    chain.joint(i_idx).v = [0,0,0]';
    chain.joint(i_idx).w = [0,0,0]';
end

% Parse link information
chain.n_link = length(robot.link); % number of link
chain.link_names = cell(1,chain.n_link);
for i_idx = 1:chain.n_link
    link_i = robot.link{i_idx};
    chain.link(i_idx).name = link_i.Attributes.name; % link name
    chain.link_names{i_idx} = chain.link(i_idx).name;
    % Get the parent joint index of the current link
    chain.link(i_idx).joint_idx = idx_cell(chain.child_link_names,chain.link(i_idx).name);
    % Parse link mesh offset
    try
        rpy = str2num(link_i.visual.origin.Attributes.rpy)';
        xyz = str2num(link_i.visual.origin.Attributes.xyz)';
    catch
        rpy = [0,0,0]';
        xyz = [0,0,0]';
    end
    p_offset = xyz;
    R_offset = rpy2r(rpy);
    chain.link(i_idx).p_offset = p_offset;
    chain.link(i_idx).R_offset = R_offset;
    % Parse link mesh scale
    try
        scale = str2num(link_i.visual.geometry.mesh.Attributes.scale)';
    catch
        scale = [1,1,1]';
    end
    chain.link(i_idx).scale = scale;
    % Parse link mesh path
    try
        [~,name,ext] = fileparts(link_i.visual.geometry.mesh.Attributes.filename);
        [f,~,~] = fileparts(urdf_path);
        mesh_path = [f,'/visual/',name,ext];
    catch
        mesh_path = '';
    end
    chain.link(i_idx).mesh_path = mesh_path;
    chain.link(i_idx).fv = '';
    if exist(chain.link(i_idx).mesh_path,'file')
        [~,~,ext] = fileparts(chain.link(i_idx).mesh_path);
        switch lower(ext)
            case '.stl'
                fv = load_stl(chain.link(i_idx).mesh_path);
                [fv.vertices,fv.faces]= patchslim(fv.vertices,fv.faces); % patch slim
                fv = reducepatch(fv,1/2); % reduce half
                fv.vertices = chain.link(i_idx).scale'.*fv.vertices;
                fv.vertices = fv.vertices * chain.link(i_idx).R_offset'; % rotate mesh locally
                fv.vertices = fv.vertices + chain.link(i_idx).p_offset'; % translate mesh
                chain.link(i_idx).fv = fv;
            otherwise
                fprintf(2,'Unsupported file type:[%s].\n',ext);
        end
    end
    % Get CAD Bounding Cube
    if ~isempty(chain.link(i_idx).fv)
        fv = chain.link(i_idx).fv;
        bcube.xyz_min = min(fv.vertices)';
        bcube.xyz_max = max(fv.vertices)';
        bcube.xyz_len = bcube.xyz_max - bcube.xyz_min;
        bcube.c_offset = 0.5*(bcube.xyz_max + bcube.xyz_min); % offset of CoM of this link
    else
        bcube = '';
    end
    chain.link(i_idx).bcube = bcube;
    % Parse link box
    try
        box = str2num(link_i.visual.geometry.box.Attributes.size)';
    catch
        box = '';
    end
    try
        box_scale = str2num(link_i.visual.geometry.box.Attributes.scale)';
    catch
        box_scale = [1,1,1]';
    end
    chain.link(i_idx).box = box;
    chain.link(i_idx).box_scale = box_scale;
end

% Determine parent/child structure
for i_idx = 1:chain.n_joint
    parent = idx_cell(chain.child_link_names,chain.joint(i_idx).parent_link);
    childs = idx_cell_multiple(chain.parent_link_names,chain.joint(i_idx).child_link);
    chain.joint(i_idx).parent = parent;
    chain.joint(i_idx).childs = childs;
end

% Zero positions
chain = update_chain_q(chain,chain.rev_joint_names,zeros(1,chain.n_rev_joint));
chain = fk_chain(chain);

% Get size of the model
xyz_min = inf*ones(3,1);
xyz_max = -inf*ones(3,1);
for i_idx = 1:chain.n_joint
    xyz_min = min(xyz_min,chain.joint(i_idx).p);
    xyz_max = max(xyz_max,chain.joint(i_idx).p);
end
xyz_len = xyz_max - xyz_min;
chain.xyz_min = xyz_min;
chain.xyz_max = xyz_max;
chain.xyz_len = xyz_len;

% Other informations
chain.rev_joint_idxs = zeros(length(chain.rev_joint_names),1);
for i_idx = 1:length(chain.rev_joint_names)
    chain.rev_joint_idxs(i_idx) = idx_cell(chain.joint_names,chain.rev_joint_names{i_idx});
end

% Center the robot
if CENTER_ROBOT
    chain.joint(chain.joint(get_topmost_idx(chain)).childs).p_offset = [0,0,0]';
    chain = update_chain_q(chain,...
        chain.rev_joint_names,0*ones(1,chain.n_rev_joint)*pi/180);
    chain = fk_chain(chain);
end

