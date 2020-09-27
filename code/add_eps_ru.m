function [t_anchor,x_anchor,l_anchor] = add_eps_ru(t_anchor,x_anchor,l_anchor,eps)
%
% Add epsilon run-up to GRP
%

n = length(t_anchor);

t_init = t_anchor(1);
t_btw = t_anchor(2:end-1);
t_final = t_anchor(end);
t_len = t_final - t_init;

x_init = x_anchor(1,:);
x_btw = x_anchor(2:end-1,:);
x_final = x_anchor(end,:);
x_diff = x_final - x_init;

l_init = l_anchor(1);
l_btw = l_anchor(2:end-1);
l_final = l_anchor(end);

t_anchor = [t_init; t_init+eps; t_btw; t_final-eps; t_final];

x_anchor = [x_init; x_init+eps*x_diff/t_len; x_btw; x_final-eps*x_diff/t_len; x_final];

l_anchor = [l_init; 1; l_btw; 1; l_final];
