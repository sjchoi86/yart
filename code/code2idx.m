function idx = code2idx(code,max_code)
%
% Convert a vectorized code to a scalar index
%

D = length(max_code); % # of dimensions of code 
max_code = max_code + 1; % max_code starts from 0

idx = 0;
for d = 1:D
    idx = idx + code(d)*prod(max_code(d:(D-1)));
end