function tk = print_tk(tk,i,n_total,n_print)
%
% Time keeping print
%

if nargin == 3
    n_print = 10;
end

rate = i/n_total;
if (mod(i,round(n_total/n_print))==0) || (i==1)
    esec = etime(clock,tk.iclk);
    esec_diff = esec - tk.esec_prev;
    fprintf('[%s][%d/%d][%.1f]%% esec:[%.1f]s esec_diff:[%.2f]s remain:[%.1f]min. \n',...
        tk.info,i,n_total,100*i/n_total,esec,esec_diff,esec/rate*(1-rate)/60);
    tk.esec_prev = esec;
    tk.PRINT = 1;
else
    tk.PRINT = 0;
end

esec_onestep = etime(clock,tk.iclk);
tk.esec_onestep_diff = esec_onestep - tk.esec_onestep_prev;
tk.esec_onestep_prev = esec_onestep;
