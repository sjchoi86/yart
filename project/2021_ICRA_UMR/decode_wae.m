function x = decode_wae(W,z)

net = z;
for h_idx = 1:numel(W.hdims_P)
    ndim = size(net,2);
    hdim = W.hdims_P(h_idx);
    % Parse parameters
    K = getfield(W, sprintf('P_hid_P_lin_%d_kernel',h_idx-1));
    b = getfield(W, sprintf('P_hid_P_lin_%d_bias',h_idx-1));
    actv = W.actv_P;
    % Linear
    identity = net;
    net = ff_net(net,K,b,'none');
    % Activation
    net = feval(actv,net);
end
K = getfield(W,'P_x_recon_kernel');
b = getfield(W,'P_x_recon_bias');
net = ff_net(net,K,b,'none');
x = net;
