function input = sample_bocd(bocd)
%
% Sample from the Bayeisan optimiztaion structure
%

input = zeros(1,bocd.d);
for i = 1:bocd.d
    switch bocd.ranges(i).type
        case 'continuous'
            input(i) = bocd.ranges(i).range(1)+...
                (bocd.ranges(i).range(end)-bocd.ranges(i).range(1))*rand;
        case 'discrete'
            n_select = length(bocd.ranges(i).range);
            input(i) = bocd.ranges(i).range(randi([1,n_select]));
        otherwise
            fprintf(2,'[sample_bo] unkown [%s].\n',bocd.ranges(i).type);
    end
end
