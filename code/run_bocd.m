function bocd = run_bocd(bocd)
%
% Run BOCD
%

switch bocd.mode
    case 'BO' % Bayesian optimization
        input = get_input_for_bocd(bocd);
        cost = bocd.f(input);
        bocd = append_bocd(bocd,input,cost);
        % append
        bocd.bo_step = bocd.bo_step + 1;
        bocd.inputs_bo(bocd.bo_step,:) = input;
        bocd.costs_bo(bocd.bo_step,:) = cost;
        % transition
        if bocd.bo_step >= bocd.max_bo_step
            bocd.bo_step = 0;
            bocd.cd_step = 0;
            bocd.mode = 'CD';
        end
        % increase iteration
        bocd.iter = bocd.iter + 1;
    case 'CD' % coordinate descend
        if bocd.max_cd_step == 0
            bocd.mode = 'BO';
            return;
        end
        if bocd.cd_step == 0 % when enters this state for the first time
            [bocd.cost_cd,min_idx] = min(bocd.costs_bo);
            bocd.input_cd = bocd.inputs_bo(min_idx,:); % initial input
        end
        d_seq = randperm(bocd.d); % randomized order
        for d_idx = d_seq % for each input dimension, check update 
            if (bocd.ranges(d_idx).range(end)-bocd.ranges(d_idx).range(1)) == 0
                % skip if the range is zero
                continue;
            end
            % terminate
            if bocd_finished(bocd)
                break;
            end
            input_cd_temp = bocd.input_cd;
            inputs_cd = zeros(bocd.n_bin,bocd.d);
            costs_cd = zeros(bocd.n_bin,1);
            for i_idx = 1:bocd.n_bin % for each bin
                if i_idx == 1
                    input = bocd.input_cd;
                else
                    input = sample_bocd(bocd); % sample 
                end
                input_cd_temp(d_idx) = input(d_idx); % update current dimension only
                cost = bocd.f(input_cd_temp); % check cost
                inputs_cd(i_idx,:) = input_cd_temp;
                costs_cd(i_idx) = cost;
            end
            [min_cost_d,min_idx] = min(costs_cd); % get min cost 
            if min_cost_d < bocd.cost_cd
                bocd.cost_cd = min_cost_d;
                bocd.input_cd = inputs_cd(min_idx,:); % update 'bocd.input_cd'
            else
                % break; % let's not stop here.. other dimensions can be useful. 
            end
        end
        % append
        bocd = append_bocd(bocd,bocd.input_cd,bocd.cost_cd);
        % increase iteration
        bocd.iter = bocd.iter + 1;
        % transition
        bocd.cd_step = bocd.cd_step + 1;
        if bocd.cd_step >= bocd.max_cd_step
            bocd.bo_step = 0;
            bocd.cd_step = 0;
            bocd.mode = 'BO';
        end
end
if bocd.VERBOSE
    print_bocd(bocd);
end