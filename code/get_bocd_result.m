function [best_input,best_cost] = get_bocd_result(bocd)
%
% Get the best input and cost of BOCD
%
[best_cost,min_idx] = min(bocd.costs(1:bocd.n));
best_input = bocd.inputs(min_idx,:);
