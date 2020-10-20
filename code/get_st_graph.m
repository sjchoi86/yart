function [st,t_test] = get_st_graph(x_anchor)
%
% Get st (i.e., s(t))
%

n_anchor = size(x_anchor,1);
t_anchor = linspace(0,1,n_anchor)';
l_anchor = ones(n_anchor,1);

% Epsilon runup
% eps_ru = 0.01;
eps_ru = ''; % no epsilon runup

% Test data
n_test = 1000;
t_test = linspace(0,1,n_test)';

% Hyper parameters
hyp_mu = [1,0.1]; % [gain,len]
hyp_var = [0.1,1.0]; % [gain,len]

% Initialize GRP
grp1d = init_grp1d(t_anchor,x_anchor,l_anchor,t_test,hyp_mu,hyp_var,'eps_ru',eps_ru);
% plot_grp1d(grp1d,'','fig_pos',[0.0,0.7,0.5,0.3]);

% Get phase
W = grp1d.mu_test;
E = exp(W);
st = cumsum(E)/sum(E);
