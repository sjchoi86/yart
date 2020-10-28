function M = parse_mlp(mat_path)
%
% Parse WAE
%

% Load
l = load(mat_path);
l_names = fieldnames(l);
M = struct();
for i_idx = 1:numel(l_names) % For all elements
    l_name = l_names{i_idx};
    val = getfield(l,l_name);
    if isnumeric(val) % Make numbers double
        val = double(val);
    end
    M = setfield(M,l_name,val);
end


% Validation
x_vald = M.x_vald;
y = mlp(M,x_vald);
y_error = mean(abs(y-M.y_vald),[1,2]);

if y_error > 0.01
    fprintf(2,'y_error:[%.3e] is too high! [%s].\n',y_error,mat_path);
end