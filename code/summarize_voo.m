function voo = summarize_voo(voo)

voo.x_list = voo.x_list(1:voo.tick,:); % in case of sudden stop 
voo.f_list = voo.f_list(1:voo.tick); 
