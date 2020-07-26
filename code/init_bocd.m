function bocd = init_bocd(ranges,f,varargin)

% Parse options
p = inputParser;
addParameter(p,'max_iter',500);
addParameter(p,'n_sample_for_acq',100);
addParameter(p,'n_burnin',50);
addParameter(p,'eps_prob',0.2);
addParameter(p,'max_save',500);
addParameter(p,'n_bin',50);
addParameter(p,'max_bo_step',10);
addParameter(p,'max_cd_step',10);
addParameter(p,'min_cost',inf);
addParameter(p,'rseed','');
addParameter(p,'VERBOSE',1);
parse(p,varargin{:});
max_iter = p.Results.max_iter;
n_sample_for_acq = p.Results.n_sample_for_acq;
n_burnin = p.Results.n_burnin;
eps_prob = p.Results.eps_prob;
max_save = p.Results.max_save;
n_bin = p.Results.n_bin;
max_bo_step = p.Results.max_bo_step;
max_cd_step = p.Results.max_cd_step;
min_cost = p.Results.min_cost;
rseed = p.Results.rseed;
VERBOSE = p.Results.VERBOSE;


% fix random seed
if ~isempty(rseed)
    rng(rseed);
end

% define BOCD
bocd.ranges = ranges; % input ranges
bocd.f = f; % cost function 

% variables
bocd.iter = 0;
bocd.max_iter = max_iter;
bocd.d = length(bocd.ranges); % input dimension

% BO configurations
bocd.n_sample_for_acq = n_sample_for_acq; % number of samples of the acquisition
bocd.n_burnin = n_burnin; % number of burn-ins
bocd.eps_prob = eps_prob; % epsilon randomness at Bayesian optimization
bocd.max_save = max_save; % maximum number of inputs to save for Bayesian optimization

% CD congigurations
bocd.n_bin = n_bin; % number of samples
bocd.input_cd = zeros(1,bocd.d); % input to check
bocd.cost_cd = 0;

% BOCD configurations
bocd.mode = 'BO'; % current mode {'BO' / 'CD'}
bocd.bo_step = 0;
bocd.cd_step = 0;
bocd.max_bo_step = max_bo_step; % number of BO steps for starting CD
bocd.max_cd_step = max_cd_step; % number of CD steps
bocd.min_cost = min_cost;

% Variables
bocd.n = 0; % number of data
bocd.inputs = zeros(bocd.max_save,bocd.d);
bocd.costs = inf*ones(bocd.max_save,1);
bocd.cost_best = inf;
bocd.inputs_bo = zeros(bocd.max_bo_step,bocd.d);
bocd.costs_bo = inf*ones(bocd.max_bo_step,1);
bocd.input_added = zeros(1,bocd.d); % input just added
bocd.cost_added = zeros(1,bocd.d); % cost just added

% Etc
bocd.VERBOSE = VERBOSE;


