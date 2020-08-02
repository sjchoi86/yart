function ik = init_ik(chain,varargin)
%
% Initialize IK
%
D2R = pi/180;

% Parse options
p = inputParser;
addParameter(p,'joint_names_control',chain.rev_joint_names);
addParameter(p,'stepsize',0.5*D2R);
addParameter(p,'stepsize_min',0.1*D2R);
addParameter(p,'stepsize_max',5.0*D2R);
addParameter(p,'stepsize_inc_rate',1.1);
addParameter(p,'stepsize_dec_rate',0.5);
addParameter(p,'max_tick',1e4);
parse(p,varargin{:});
joint_names_control = p.Results.joint_names_control;
stepsize = p.Results.stepsize;
stepsize_min = p.Results.stepsize_min;
stepsize_max = p.Results.stepsize_max;
stepsize_inc_rate = p.Results.stepsize_inc_rate;
stepsize_dec_rate = p.Results.stepsize_dec_rate;
max_tick = p.Results.max_tick;

% Initialize IK
ik.chain = chain; % save the kinematic chain structure
ik.joint_names_control = joint_names_control; % joints to control
ik.joint_idxs_control = zeros(1,length(joint_names_control));
for i_idx = 1:length(joint_names_control)
    ik.joint_idxs_control(i_idx) = idx_cell(chain.joint_names,ik.joint_names_control{i_idx});
end
ik.n_joint_control = length(ik.joint_names_control);
ik.tick = 0;
ik.tick_dec = 0;
ik.err = inf;
ik.err_prev = 0;
ik.err_diff = 0;
ik.stepsize = stepsize;
ik.stepsize_min = stepsize_min;
ik.stepsize_max = stepsize_max;
ik.stepsize_inc_rate = stepsize_inc_rate;
ik.stepsize_dec_rate = stepsize_dec_rate;

ik.max_tick = max_tick;
ik.q_list = zeros(ik.max_tick,ik.n_joint_control); % q list 
ik.err_list = inf*ones(1,ik.max_tick);
ik.err_diff_list = inf*ones(1,ik.max_tick);

% Joint limits
ik.joint_limits_lower = zeros(ik.n_joint_control,1);
ik.joint_limits_upper = zeros(ik.n_joint_control,1);
for i_idx = 1:ik.n_joint_control
    limit = ik.chain.joint(idx_cell(ik.chain.joint_names,ik.joint_names_control{i_idx})).limit;
    ik.joint_limits_lower(i_idx) = limit(1);
    ik.joint_limits_upper(i_idx) = limit(2);
end

% IK targets
ik.n_target = 0;
ik.targets = [];

% IK position and rotation weights
ik_p_weight = 40.0/max(chain.xyz_len);
ik_R_weight = 0.05;
ik.W_full = diag([ik_p_weight,ik_p_weight,ik_p_weight,...
    ik_R_weight,ik_R_weight,ik_R_weight]);
