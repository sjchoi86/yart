function [x_start, y_start, x_len, y_len] ...
    = subaxes(figHandler, h, w, index, xm, ym)
%
% Subfigure with controllable margin 
%
if nargin == 4
    xm = 0.10;
    ym = 0.15;
end

w_idx = rem(index, w);
if w_idx == 0, w_idx = w; end
w_idx = w_idx - 1;
h_idx = ceil(index/w) - 1;
h_idx = h - h_idx - 1;

w_unit = 1/w;
h_unit = 1/h;

x_start = w_unit*(w_idx) + xm/2;
y_start = h_unit*(h_idx) + ym/2;
x_len = w_unit-xm;
y_len = h_unit-ym;

axes('Parent', figHandler ...
    , 'Position' ...
    , [x_start, y_start, x_len, y_len] );      
