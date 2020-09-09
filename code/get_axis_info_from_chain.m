function axis_info = get_axis_info_from_chain(ws,chain_model_sz,varargin)
%
% Get axis information from JOI workspaces
%

% Parse options
p = inputParser;
addParameter(p,'margin_rate',0.0); % 0.0~
parse(p,varargin{:});
margin_rate = p.Results.margin_rate;



xyz_min = inf*[1,1,1]; xyz_max = -inf*[1,1,1];
for w_idx = 1:ws.n
    xyz_min = min(xyz_min,ws.info{w_idx}.min_vals');
    xyz_max = max(xyz_max,ws.info{w_idx}.max_vals');
end
axis_info = [xyz_min(1),xyz_max(1),xyz_min(2),xyz_max(2),xyz_min(3),xyz_max(3)];
axis_info(1) = min(axis_info(1),chain_model_sz.xyz_min(1));
axis_info(2) = max(axis_info(2),chain_model_sz.xyz_max(1));
axis_info(3) = min(axis_info(3),chain_model_sz.xyz_min(2));
axis_info(4) = max(axis_info(4),chain_model_sz.xyz_max(2));
axis_info(5) = chain_model_sz.xyz_min(3);

% Give margins
x_range = axis_info(2) - axis_info(1);
y_range = axis_info(4) - axis_info(3);
z_range = axis_info(6) - axis_info(5);
axis_info(1) = axis_info(1) - 0.5*margin_rate*x_range;
axis_info(2) = axis_info(2) + 0.5*margin_rate*x_range;
axis_info(3) = axis_info(3) - 0.5*margin_rate*y_range;
axis_info(4) = axis_info(4) + 0.5*margin_rate*y_range;
axis_info(6) = axis_info(6) + 0.5*margin_rate*z_range;










