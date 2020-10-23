function z = encode_wae(W,x)

net = x;
for h_idx = 1:numel(W.hdims_Q)
    ndim = size(net,2);
    hdim = W.hdims_Q(h_idx);
    % Parse parameters
    K = getfield(W, sprintf('Q_hid_Q_lin_%d_kernel',h_idx-1));
    b = getfield(W, sprintf('Q_hid_Q_lin_%d_bias',h_idx-1));
    actv = W.actv_Q;
    % Linear
    identity = net;
    net = ff_net(net,K,b,'none');
    % Activation
    net = feval(actv,net);
end
K = getfield(W,'Q_z_real_kernel');
b = getfield(W,'Q_z_real_bias');
net = ff_net(net,K,b,'none');
z = net;
