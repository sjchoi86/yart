function mm_info = get_minmax(mm_info,pos)
%
% Get minimum and maximum points
%

if isnan(sum(pos(:)))
    return;
end

if isempty(mm_info)
    mm_info.min_vals = inf*ones(size(pos));
    mm_info.max_vals = -inf*ones(size(pos));
else
    if ~isempty(pos)
        mm_info.min_vals = min(mm_info.min_vals,pos);
        mm_info.max_vals = max(mm_info.max_vals,pos);
    end
end

mm_info.len_vals = mm_info.max_vals-mm_info.min_vals;
