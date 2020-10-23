function W = parse_wae(mat_path)
%
% Parse WAE
%

% Load
l = load(mat_path);
l_names = fieldnames(l);
W = struct();
for i_idx = 1:numel(l_names) % For all elements
    l_name = l_names{i_idx};
    val = getfield(l,l_name);
    if isnumeric(val) % Make numbers double
        val = double(val);
    end
    W = setfield(W,l_name,val);
end

% Validation
x_real_vald = W.x_real_vald;
z_real = encode_wae(W,x_real_vald);
x_recon = decode_wae(W,z_real);
z_vald = W.z_real_vald;
x_recon_vald = W.x_recon_vald;
z_error = mean(abs(z_real-z_vald),[1,2]);
x_error = mean(abs(x_recon-x_recon_vald),[1,2]);

if z_error > 0.01
    fprintf(2,'z_error:[%.3e] is too high! [%s].\n',z_error,mat_path);
end
if x_error > 0.01
    fprintf(2,'x_error:[%.3e] is too high! [%s].\n',x_error,mat_path);
end
