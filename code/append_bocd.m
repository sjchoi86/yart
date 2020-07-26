function bocd = append_bocd(bocd,input,cost)
%
% Add current data to the Bayesian optimization structure
%

bocd.n = bocd.n + 1;
bocd.inputs(bocd.n,:) = input;
bocd.costs(bocd.n,:) = cost; % the smaller, the better

% some heuristics
max_save = bocd.max_save;
if bocd.n > max_save
    [~,sort_idx] = sort(bocd.costs(1:bocd.n,:));
    X = bocd.inputs(sort_idx(1:max_save), :);
    KX = kernel_ardmatern_bocd(X, X, bocd);
    bocd.KX = KX;
    
    bocd.n = max_save;
    bocd.inputs = bocd.inputs(sort_idx(1:max_save), :);
    bocd.costs = bocd.costs(sort_idx(1:max_save));
else
    X = bocd.inputs(1:bocd.n, :);
    KX = kernel_ardmatern_bocd(X, X, bocd);
    bocd.KX = KX;
end

% track the best cost 
if cost < bocd.cost_best
    bocd.cost_best = cost;
end

% just added
bocd.input_added = input;
bocd.cost_added = cost;
