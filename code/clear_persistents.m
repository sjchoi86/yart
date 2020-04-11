function clear_persistents

% Clear all persistent variables in functions start with 'plot_' such as 'plot_T'
plot_func_names = dir('../**/plot_*.m');
for i_idx = 1:length(plot_func_names)
    plot_func_name = plot_func_names(i_idx).name;
    clear(plot_func_name(1:end-2));
end
