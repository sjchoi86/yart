function tk = init_tk(info_str)
%
% tk = print_tk('name');
% while FLAG
%   tk = print_tk(tk,idx,n_total);
% end
%
tk.info = info_str;
tk.iclk = clock;
tk.esec_prev = 0;

tk.esec_onestep_prev = 0;
