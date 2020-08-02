function out = slerp(a,b,t)
%
% Slerp (spherical interpolation)
%
Omega = acos(a'*b)/norm(a)/norm(b);
out = sin((1-t)*Omega)/sin(Omega)*a + sin(t*Omega)/sin(Omega)*b;
