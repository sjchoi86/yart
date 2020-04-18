function childs = get_childs_from_parents(parents)
%
% Get childs fields from parent field
%
childs = cell(1,length(parents));
for i_idx = 1:length(parents)
    if ~isempty(parents{i_idx})
        parent = parents{i_idx};
        if parent > 0
            childs{parent} = [childs{parent}, i_idx];
        end
    end
end
