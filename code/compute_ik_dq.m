function dq = compute_ik_dq(J,ik_err,ik_W_err)
%
% Damped Least-square IK
%
n_ctrl = size(J,2);
lambda = 0.5;
Jh = J'*ik_W_err*J + lambda*eye(n_ctrl,n_ctrl);
g_err = J'*ik_W_err*ik_err;
dq = Jh \ g_err * 1.0;
