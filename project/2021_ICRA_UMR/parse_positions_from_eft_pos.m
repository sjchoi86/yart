function [p_root,p_spine,p_neck,p_neck2,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_head,p_rp,p_lp] = ...
    parse_positions_from_eft_pos(vec)
%
% Parse joint positions from EFT pos
%
p_root = vec(1,:);
p_spine = vec(11,:);
p_neck = vec(13,:);
p_neck2 = vec(12,:);
p_rs = vec(4,:);
p_re = vec(3,:);
p_rh = vec(2,:);
p_ls = vec(7,:);
p_le = vec(6,:);
p_lh = vec(5,:);
p_head = vec(10,:);
p_rp = vec(8,:);
p_lp = vec(9,:);