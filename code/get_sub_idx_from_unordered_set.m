function [sub_idx,K] = get_sub_idx_from_unordered_set(x,k,varargin)

% Parse options
p = inputParser;
addParameter(p,'k_gain',500);
addParameter(p,'rand_gain',0);
addParameter(p,'VERBOSE',0);
parse(p,varargin{:});
rand_gain = p.Results.rand_gain;
VERBOSE = p.Results.VERBOSE;
k_gain = p.Results.k_gain;


n = size(x,1);
nzr_x = init_nz(x);
nzd_x = get_nzdval(nzr_x,x);
p = pdist(nzd_x,'squaredeuclidean');
D = squareform(p); % get pairwise distances

K = exp(-k_gain*D); % kernel-like matrix

% L-ensemble in k-DPP
% K = eye(n,n)-inv(K+eye(n,n));

K = 0.5*K + 0.5*K';

remain_idxs = 1:n;
sub_idx = zeros(k,1);
sum_K_vec = zeros(1,n);
for k_idx = 1:k
    if k_idx == 1
        sel_idx = randi([1,n]); % random select initially
        % sel_idx = 1;
    else
        %
        % Let K be a kernel matrix. 
        % Supoose we have 10 items, [1,2,3,...,10], and currently, [2,4] are selected.
        % Then for i in [1,3,5,6,7,8,9,10], 
        %   k_sum_i = sum(K(i,[2,4])) <= this becomes a proxy for how far i-th data is compared to existing subset, [2,4].
        % and select the index whose k_sum_i is the smallest
        %
        curr_K_vec = K(sub_idx(k_idx-1),:); 
        sum_K_vec = sum_K_vec + curr_K_vec; % sum of kernel values
        k_vals = sum_K_vec(1,remain_idxs);
        
        rand_vec = rand_gain*max(k_vals)*randn(1,length(k_vals)); % add sum randomness
        [~,min_idx] = min(k_vals+rand_vec); 
        sel_idx = remain_idxs(min_idx);
    end
    sub_idx(k_idx) = sel_idx; % append 'sel_idx' to 'sel_idxs'
    
    remain_idxs(remain_idxs==sel_idx) = []; % remove 'sel_idx' from 'remain_idxs'
    
    if VERBOSE
        fprintf('[%d/%d]\n',k_idx,k);
    end
end


