function chain_model = update_chain_dynamics(chain_model,joints,q_rad_diff,T,varargin)
%
% Update simple dynamics
%

% Parse options
p = inputParser;
addParameter(p,'density',2710/4); % density of alumninium is 2710 [kg/m^3] / steel is 8050 [kg/m^3]
parse(p,varargin{:});
density = p.Results.density;

for i_idx = 1:chain_model.n_link % for all link 
    
    capsule_i = chain_model.link(i_idx).capsule;
    if isempty(capsule_i), continue; end
    
    joint_idx_i = chain_model.link(i_idx).joint_idx;
    p_i = chain_model.joint(joint_idx_i).p;
    R_i = chain_model.joint(joint_idx_i).R;
    cline_i = get_line_capsule(p_i,R_i,capsule_i);
    radius_i = capsule_i.radius;
    height_i = capsule_i.height;
    
    % Center of mass of each link 
    chain_model.link(i_idx).com = 0.5*(cline_i.e1+cline_i.e2)';
    % Mass of each link
    mass_i = density*((4/3)*pi*(radius_i^3)+pi*(radius_i^2)*height_i);
    if mass_i > 30
        % mass_i = 0;
    end
    chain_model.link(i_idx).mass = mass_i;
    % Moment of inertia 
    u1 = [0,1,0]';
    u2 = cline_i.e2-cline_i.e1; u2 = u2 / norm(u2);
    R = get_r_a_to_b(u1,u2);
    r = radius_i;
    m = mass_i;
    h = height_i;
    ixx = m*h*(1/8)*(3*r+2*h);
    izz = ixx;
    iyy = (2/5)*m*r*r;
    Ibar = diag([ixx,iyy,izz]);
    I_i = R*Ibar*R';
    chain_model.link(i_idx).I = I_i;
end

% Compute total CoM position
M = 0; MC = [0,0,0];
for i_idx = 1:chain_model.n_link
    if isempty(chain_model.link(i_idx).mass)
        continue;
    end
    M = M + chain_model.link(i_idx).mass;
    MC = MC + chain_model.link(i_idx).mass*chain_model.link(i_idx).com;
end
chain_model.M = M;
chain_model.com = MC / M;

% Compute linear momentum
chain_model.P = calc_model_p(chain_model,get_topmost_idx(chain_model));

% Compute angular momentum
chain_model.L = calc_model_l(chain_model,get_topmost_idx(chain_model));







function R = get_r_a_to_b(a,b)
%
% R from a to b
%
a = reshape(a, [3,1]);
b = reshape(b, [3,1]);
a = a / norm(a);
b = b / norm(b);
v = cross(a,b); 
ssc = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0]; 
R = eye(3) + ssc + ssc^2*(1-dot(a,b))/(norm(v))^2;

function P = calc_model_p(chain_model,idx_to)
%
% P
%
if isempty(idx_to)
    P = 0;
else
    link_idx = idx_cell(chain_model.link_names,chain_model.joint(idx_to).child_link);
    c = chain_model.link(link_idx).com;
    m = chain_model.link(link_idx).mass;
    if isempty(c)
        P = 0;
    else
        P = m * (chain_model.joint(idx_to).v' + cross(chain_model.joint(idx_to).w, c));
    end
    % Recursive
    childs = chain_model.joint(idx_to).childs;
    for child = childs
        P = P + calc_model_p(chain_model,child);
    end
end

function L = calc_model_l(chain_model,idx_to)
%
% L
%
if isempty(idx_to)
    L = 0;
else
    link_idx = idx_cell(chain_model.link_names,chain_model.joint(idx_to).child_link);
    c = chain_model.link(link_idx).com;
    m = chain_model.link(link_idx).mass;
    if isempty(c)
        L = 0;
    else
        % L = m * (model.node(idx_to).v + cross(model.node(idx_to).w, c));
        P = m * (chain_model.joint(idx_to).v' + cross(chain_model.joint(idx_to).w, c));
        L = cross(c, P) + (chain_model.link(link_idx).I*chain_model.joint(idx_to).w)';
    end
    % Recursive
    childs = chain_model.joint(idx_to).childs;
    for child = childs
        L = L + calc_model_l(chain_model,child);
    end
end









