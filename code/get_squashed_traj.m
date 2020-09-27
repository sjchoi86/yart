function squashed_traj = get_squashed_traj(raw_traj,min_vals,max_vals,margin)
%
% Get squashed trajectory with soft margin
%

dim = size(raw_traj,2);
squashed_traj = raw_traj;

if margin == 0
    margin = 1e-2;
end

for d_idx = 1:dim % for each dimension 
    min_val = min_vals(d_idx);
    max_val = max_vals(d_idx);
    raw_vec = raw_traj(:,d_idx);
    
    % Lower limit
    raw_vec(raw_vec < (min_val+margin)) = ...
        alpha_squash(raw_vec(raw_vec < (min_val+margin))-(min_val+margin),margin) + ...
        (min_val+margin);
    
    % Upper limit
    raw_vec(raw_vec > (max_val-margin)) = ...
        alpha_squash(raw_vec(raw_vec > (max_val-margin))-(max_val-margin),margin) + ...
        (max_val-margin);
    
    % Append
    squashed_traj(:,d_idx) = raw_vec;
end


function y = alpha_squash(x,alpha)
%
% x -> (-alpha, +alpha)
%
y = alpha*(exp(2/alpha*x)-1)./(exp(2/alpha*x)+1);
