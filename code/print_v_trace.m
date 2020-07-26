function print_v_trace(v_trace)

v_src = v_trace(1);
v_final = v_trace(end);
fprintf('Path from [%d] to [%d] is:\n',v_src,v_final);
for v_check = v_trace
    fprintf('[%d]',v_check);
    if v_check ~= v_final
        fprintf('=>');
    end
end
fprintf('\n');
