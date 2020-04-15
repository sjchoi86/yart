function [ik,chain,q,LIMBO] = onestep_ik(ik,chain,q)
%
% Run one-step IK
%
D2R = pi/180;

ik.tick = ik.tick + 1; % increase tick

% Get IK ingredients
[J_total,ik_err_total,ik_W_total] = ik_ingredients(ik,chain,ik.joint_names_control);

% Compute dq
dq = compute_ik_dq(J_total,ik_err_total,ik_W_total); % numerical gradient
ik_err_val = norm(ik_err_total);
ik.err = ik_err_val;

% Check limbo
LIMBO = check_ik_oscilating(ik);
if LIMBO
    ik.stepsize = ik.stepsize*ik.stepsize_dec_rate;
end

% Step-size scheduling
ik.err_diff = ik.err_prev-ik.err;
if ik.err_diff > 0 % if it gets better
    ik.stepsize = ik.stepsize*ik.stepsize_inc_rate;
else
    ik.stepsize = ik.stepsize*ik.stepsize_dec_rate;
end
ik.stepsize = min(max(ik.stepsize,ik.stepsize_min),ik.stepsize_max);
ik.err_prev = ik.err;

% Pre-update q and check joint limit violation
q_check = q + ik.stepsize*dq;
is_not_violated = zeros(1,ik.n_joint_control);
for i_idx = 1:ik.n_joint_control
    margin = 1.0*D2R;
    lower = ik.joint_limits_lower(i_idx)+margin;
    upper = ik.joint_limits_upper(i_idx)-margin;
    if ((q_check(i_idx)<lower) && (dq(i_idx)<0)) || ...
            ((q_check(i_idx)>upper) && (dq(i_idx)>0))
        flag = 0;
    else
        flag = 1;
    end
    is_not_violated(i_idx) = flag;
end

% Recompute dq using valid ones
if sum(is_not_violated == 0) > 0 % if violation occured
    joint_names_control_temp = ik.joint_names_control(is_not_violated==1);
    [J_temp,ik_err_temp,ik_W_temp] = ik_ingredients(ik,chain,joint_names_control_temp);
    dq_temp = compute_ik_dq(J_temp,ik_err_temp,ik_W_temp); % numerical gradient
    dq = zeros(ik.n_joint_control,1);
    dq(is_not_violated==1) = dq_temp;
end

% Update position
dq = scale_max(dq,ik.stepsize);
q = q + dq;
q = min(max(q,ik.joint_limits_lower),ik.joint_limits_upper); % trim limits
chain = update_chain_q(chain,ik.joint_names_control,q);
chain = fk_chain(chain);

% Save
ik.q_list(ik.tick,:) = q'; % append position
ik.err_list(ik.tick) = ik.err;
ik.err_diff_list(ik.tick) = ik.err_diff; 


function [J_total,ik_err_total,ik_W_total] = ik_ingredients(ik,chain,joint_names_control)
%
% Get IK Ingredients with 'joint_names_control'
%
J_total = [];
ik_err_total = [];
ik_W_diag_total = [];
for i_idx = 1:ik.n_target
    % Compute a Jacobian Matrix
    J = compute_jacobian(chain,joint_names_control,ik.targets(i_idx).joint_name);
    % Compute an IK error
    [ik_err,ik_idx] = compute_ik_err(chain,ik.targets(i_idx).joint_name,...
        ik.targets(i_idx).p,ik.targets(i_idx).R,...
        ik.targets(i_idx).IK_P,ik.targets(i_idx).IK_R);
    J = J(ik_idx,:);
    ik_err = ik_err(ik_idx,:);
    ik_W = ik.W_full(ik_idx,ik_idx);
    % Append
    J_total = [J_total; J];
    ik_err_total = [ik_err_total; ik_err];
    ik_W_diag_total = [ik_W_diag_total; diag(ik_W)];
end
ik_W_total = diag(ik_W_diag_total);