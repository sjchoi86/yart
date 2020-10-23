function c = get_color_with_first_and_second_coordinates(x)

c = x(:,1:2);
c = (c-min(c))./(max(c)-min(c));
r = 1.0-c(:,2);
g = c(:,1);
b = 0.5-zeros(size(r));
c = [r,g,b];
