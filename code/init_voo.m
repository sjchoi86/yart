function voo = init_voo(n_sample,dim,max_exploit,omega,f,px)

voo.n_sample = n_sample;
voo.dim = dim;
voo.max_exploit = max_exploit;
voo.omega = omega;
voo.f = f;
voo.px = px;

voo.tick = 0;

voo.x_list = zeros(n_sample,dim); % this wil contain all samples from VOO
voo.f_list = zeros(1,n_sample);
