function [FLAG,ik,best_err] = check_ik_oscilating(ik)
%
% Check whether current IK is in limbo
%

FLAG = 0;
oscilation_check_tick = 10;
if ik.tick > oscilation_check_tick
    partial_ik_err_val_diff_list = ik.err_diff_list(ik.tick-(oscilation_check_tick-1):ik.tick);
    if (mean(cumsum(sign(partial_ik_err_val_diff_list))) <= 0.6) && ...
            (mean(cumsum(sign(partial_ik_err_val_diff_list))) >= 0.4)
        [~,min_idx] = min(ik.err_list);
        q = ik.q_list(min_idx,:)';
        ik.chain = update_chain_q(ik.chain,ik.joint_names_control,q);
        ik.chain = fk_chain(ik.chain);
        FLAG = 1;
    end
end
best_err = min(ik.err_list(1:ik.tick));