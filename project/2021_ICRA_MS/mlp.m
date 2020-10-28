function y = mlp(M,x)

net = x;
for h_idx = 1:numel(M.hdims)
    ndim = size(net,2);
    hdim = M.hdims(h_idx);
    % Parse parameters
    K = getfield(M, sprintf('F_hid_lin_%d_kernel',h_idx-1));
    b = getfield(M, sprintf('F_hid_lin_%d_bias',h_idx-1));
    actv = M.actv;
    % Linear
    identity = net;
    net = ff_net(net,K,b,'none');
    % Activation
    net = feval(actv,net);
end
K = getfield(M,'F_y_pred_kernel');
b = getfield(M,'F_y_pred_bias');
net = ff_net(net,K,b,'none');
y = net;
