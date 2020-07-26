function val = f_simple_cost_in_2d(x)

scale = 10;

n = size(x,1);
x1 = 4*x(:,1)/scale;
x2 = 4*x(:,2)/scale;

val = sqrt(1e-8)*randn(n,1)...
    -5*cos(0.5*pi*sqrt(x1.*x1+x2.*x2)).*exp(-0.1*pi*sqrt(x1.*x1+x2.*x2))...
    +6;
