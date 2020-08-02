function V = transform_vertex(V,p,R)

n = size(V,1);
V = V*R' + ones(n,1)*reshape(p,[1,3]);
