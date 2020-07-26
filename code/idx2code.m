function code = idx2code(idx,max_code)
%
% Convert a scalar index to a vectorized code
%

D = length(max_code); % # of dimensions of code 
max_code = max_code + 1; % max_code starts from 0

code = zeros(1,D);
for d = 1:D
    temp = prod(max_code(d:(D-1)));
    code(d) = floor(idx / temp);
    idx = idx - code(d)*temp;
end