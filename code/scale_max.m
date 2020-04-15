function out_val = scale_max(in_val,max_abs_th)
%
% Scale min and max 
%

% out_val = min(in_val,max_th);
% return;

max_abs_val = max(abs(in_val(:)));
if max_abs_val > max_abs_th
    out_val = in_val/max_abs_val*max_abs_th;
else
    out_val = in_val;
end
