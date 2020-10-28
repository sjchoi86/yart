addpath_code
ccc
%%
ccc

% Configuration
model_name = 'alphred';

% Parse trained MLP
mat_path = sprintf('script/nets/mlp_%s/weights.mat',model_name);
M = parse_mlp(mat_path);

% Get Data
dirs = dir(sprintf('data/%s-dataset/*.mat',model_name));
x_data = []; y_data = [];
for i_idx = 1:length(dirs) % for i_idx = 1:length(dirs) % for all mat files
    l = load([dirs(i_idx).folder,'/',dirs(i_idx).name]);
    x_data = [x_data; l.in_q];
    y_data = [y_data; l.out_t,l.out_w];
end % for i_idx = 1:length(dirs) % for all mat files
n = size(x_data,1);
fprintf('[%s] data.\n',n);

% Check
y_pred = mlp(M,x_data);
err = mean(abs(y_pred-y_data));

%% Plot
ca; % close all
ymin = min(y_data(:)); ymax = max(y_data(:));

figure(1); imagesc(y_data); colorbar; colormap jet; caxis([ymin,ymax]);
title('y_data','interpreter','none'); 

figure(2); imagesc(y_pred); colorbar; colormap jet; caxis([ymin,ymax]);
title('y_pred','interpreter','none'); 

figure(3); imagesc(y_pred-y_data); colorbar; colormap jet; caxis([-2,+2]);
title('y_pred-y_data','interpreter','none'); 


