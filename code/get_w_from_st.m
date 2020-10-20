function w = get_w_from_st(st)
%
% Get w(t) from the normalized s(t)
%

% Find corresponding w(t)
st_diff = [st(1); st(2:end)-st(1:end-1)];
Z = 1; % it doesn't really matter which normalization constant we are using
w_bar = Z*st_diff; 
w = log(w_bar);
w = w - mean(w); % mean zero
