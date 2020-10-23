function y = leaky_relu(x)

y = zeros(size(x));
y(x > 0) = x(x > 0);
y(x <= 0) = 0.2*x(x <= 0);
