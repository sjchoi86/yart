function idx_list = idx_cell_multiple(cell_data,query_data)
%
% Find the index whose corresponding item matches the 'query_data'
% We assume that idx_list can be an array. 
%

% query_data
idx_list = ind2sub(size(cell_data),find(cellfun(@(x)strcmp(x,query_data),cell_data)));