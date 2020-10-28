function net = ff_net(net,K,b,actv)

% Weight
net = net * K;
% Bias
net = net + repmat(b,[size(net,1),1]);
% Activation
switch actv
    case 'tanh'
        net = tanh(net);
    case 'none'
        net = net;
    case 'softmax'
        net = softmax(net,2);
    case 'relu'
        net = relu(net);
    case 'leaky_relu'
        net = leaky_relu(net);
    otherwise
        fprintf(2,'[ff_net] Unknown activation function:[%s].\n',actv);
end
