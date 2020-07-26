function input = get_input_for_bocd(bocd)
%
% Make an acquisition
%

if (rand < bocd.eps_prob) || (bocd.n < bocd.n_burnin)
    % random sample
    input = sample_bocd(bocd);
else
    % make acquisition
    inputs = zeros(bocd.n_sample_for_acq,bocd.d);
    acq_vals = zeros(bocd.n_sample_for_acq, 1);
    for i = 1:bocd.n_sample_for_acq
        input = sample_bocd(bocd);
        acquisition_val = acquisition_for_bocd(input,bocd);
        inputs(i,:)  = input;
        acq_vals(i) = acquisition_val;
    end
    [~,max_idx] = max(acq_vals);
    input = inputs(max_idx, :);
end
