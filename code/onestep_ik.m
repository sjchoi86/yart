function [ik,chain,q,LIMBO] = onestep_ik(ik,chain,q,IGNORE_LIMIT)
%
% Run one-step IK
%
global TIME_PROFILING;

if nargin == 3
    IGNORE_LIMIT = 0;
end

D2R = pi/180;
ik.tick = ik.tick + 1; % increase tick
ems = [];

% Get IK ingredients
iclk = clock;
[J_total,ik_err_total,ik_W_total] = ...
    ik_ingredients(ik,chain,ik.joint_names_control,ik.joint_idxs_control);
if TIME_PROFILING
    ems.ik_ingredient = etime(clock,iclk)*1000;
end

% Compute dq
iclk = clock;
dq = compute_ik_dq(J_total,ik_err_total,ik_W_total); % numerical gradient
if TIME_PROFILING
    ems.dq = etime(clock,iclk)*1000;
end
ik_err_val = norm(ik_err_total);
ik.err = ik_err_val;

% Check limbo
iclk = clock;
LIMBO = check_ik_oscilating(ik);
if TIME_PROFILING
    ems.limbo = etime(clock,iclk)*1000;
end
if LIMBO
    ik.stepsize = ik.stepsize*ik.stepsize_dec_rate;
end

% Step-size scheduling
iclk = clock;
ik.err_diff = ik.err_prev-ik.err;
if ik.err_diff > 1e-4 % if it gets better
    ik.stepsize = ik.stepsize*ik.stepsize_inc_rate;
else
    ik.stepsize = ik.stepsize*ik.stepsize_dec_rate;
    ik.tick_dec = ik.tick_dec + 1; % tick for decrese
end
ik.stepsize = min(max(ik.stepsize,ik.stepsize_min),ik.stepsize_max);
ik.err_prev = ik.err;
if TIME_PROFILING
    ems.stepsize = etime(clock,iclk)*1000;
end

% Pre-update q and check joint limit violation
iclk = clock;
if IGNORE_LIMIT == 0
    is_not_violated = zeros(1,ik.n_joint_control);
    q_check = q + ik.stepsize*dq;
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
else
    is_not_violated = ones(1,ik.n_joint_control);
end
if TIME_PROFILING
    ems.violation = etime(clock,iclk)*1000;
end

% Recompute dq using valid ones
iclk = clock;
if sum(is_not_violated == 0) > 0 % if violation occured
    joint_names_control_temp = ik.joint_names_control(is_not_violated==1);
    joint_idxs_control_temp = ik.joint_idxs_control(is_not_violated==1);
    [J_temp,ik_err_temp,ik_W_temp] = ...
        ik_ingredients(ik,chain,joint_names_control_temp,joint_idxs_control_temp);
    dq_temp = compute_ik_dq(J_temp,ik_err_temp,ik_W_temp); % numerical gradient
    dq = zeros(ik.n_joint_control,1);
    dq(is_not_violated==1) = dq_temp;
end
if TIME_PROFILING
    ems.recompute_dq = etime(clock,iclk)*1000;
end

% Update position
iclk = clock;
dq = scale_max(dq,ik.stepsize);
q = q + dq;
if IGNORE_LIMIT == 0
    q = min(max(q,ik.joint_limits_lower),ik.joint_limits_upper); % trim limits
end
chain = update_chain_q(chain,ik.joint_names_control,q,'IGNORE_LIMIT',IGNORE_LIMIT);
chain = fk_chain(chain);
if TIME_PROFILING
    ems.update = etime(clock,iclk)*1000;
end

% Save
ik.q_list(ik.tick,:) = q'; % append position
ik.err_list(ik.tick) = ik.err;
ik.err_diff_list(ik.tick) = ik.err_diff;

if TIME_PROFILING
    fprintf([' [onestep_ik] Jacobian:[%.2f]ms limbocheck:[%.2f]ms stepsize:[%.2f]ms',...
        ' violation:[%.2f]ms recompute:[%.2f]ms update:[%.2f]ms.\n'],...
        ems.ik_ingredient,ems.limbo,ems.stepsize,...
        ems.violation,ems.recompute_dq,ems.update);
end










function [J_total,ik_err_total,ik_W_total,ems] = ...
    ik_ingredients(ik,chain,joint_names_control,joint_idxs_control)
%
% Get IK Ingredients with 'joint_names_control'
%
global TIME_PROFILING
ems.jacobian = 0;
ems.ikerr = 0;
ems.append = 0;
J_total = [];
ik_err_total = [];
ik_W_diag_total = [];
for i_idx = 1:ik.n_target
    % Compute a Jacobian Matrix
    iclk = clock;
    J = compute_jacobian(chain,joint_names_control,joint_idxs_control,ik.targets(i_idx).joint_name);
    ems.jacobian = ems.jacobian + etime(clock,iclk)*1000;
    % Compute an IK error
    iclk = clock;
    [ik_err,ik_idx] = compute_ik_err(chain,ik.targets(i_idx).joint_name,...
        ik.targets(i_idx).p,ik.targets(i_idx).R,...
        ik.targets(i_idx).IK_P,ik.targets(i_idx).IK_R);
    J = J(ik_idx,:);
    ik_err = ik_err(ik_idx,:);
    ik_W = ik.W_full(ik_idx,ik_idx);
    ems.ikerr = ems.ikerr + etime(clock,iclk)*1000;
    % Append
    iclk = clock;
    J_total = [J_total; J];
    ik_err_total = [ik_err_total; ik_err];
    ik_W_diag_total = [ik_W_diag_total; diag(ik_W)];
    ems.append = ems.append + etime(clock,iclk)*1000;
end
ik_W_total = diag(ik_W_diag_total);
if TIME_PROFILING
    fprintf(' [onestep_ik-ik_ingredients] jacobian:[%.3f]ms ikerr:[%.3f]ms append:[%.3f]ms \n',...
        ems.jacobian,ems.ikerr,ems.append);
end
