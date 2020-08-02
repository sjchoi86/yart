function [p_neck_mr,p_rs_mr,p_re_mr,p_rh_mr,p_ls_mr,p_le_mr,p_lh_mr] = ...
    get_p_joi_motion_retarget(p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck,...
    mr_vec)
%
% Get positions of JOI for motion retargeting
% mr_vec: 
% --------------------------------------------------------------
%  (1) root to neck 'at ease' rate (0.0 ~ 1.0)
%  (2) neck to shoulder 'at ease' rate (0.0 ~ 1.0)
%  (3) shoulder to elbox 'at ease' rate (0.0 ~ 1.0)
%  (4) elbow to hand 'at ease' rate (0.0 ~ 1.0)
% --------------------------------------------------------------
%  (5) root to neck lengthen_rate (0.0 ~ inf)
%  (6) neck to shoulder lengthen_rate (0.0 ~ inf)
%  (7) shoulder to elbox lengthen_rate (0.0 ~ inf)
%  (8) elbow to hand lengthen_rate (0.0 ~ inf)
% --------------------------------------------------------------


% Reconstruct the Skeleton based on the Parametrization
root2neck = p_neck - p_root;
neck2rs = p_rs - p_neck; rs2re = p_re - p_rs; re2rh = p_rh - p_re;
neck2ls = p_ls - p_neck; ls2le = p_le - p_ls; le2lh = p_lh - p_le;
% *atease_rate: 0.0 ~ 1.0 where 1.0 corresponds to the `at ease` pose.
root2neck_atease_rate = mr_vec(1); % root to neck
neck2s_atease_rate = mr_vec(2); % neck to shoulder
s2e_atease_rate = mr_vec(3); % shoulder to elbox
e2h_atease_rate = mr_vec(4); % elbow to hand
% *lengthen_rate: length lengthening rate
root2neck_lengthen_rate = mr_vec(5); % root to neck
neck2s_lengthen_rate = mr_vec(6); % neck to shoulder
s2e_lengthen_rate = mr_vec(7); % shoulder to elbox
e2h_lengthen_rate = mr_vec(8); % elbow to hand
root2neck_mr = rotate_vec(root2neck,[0,0,1]',root2neck_atease_rate);
neck2rs_mr = rotate_vec(neck2rs,[0,-1,0]',neck2s_atease_rate);
rs2re_mr = rotate_vec(rs2re,[0,0,-1]',s2e_atease_rate);
re2rh_mr = rotate_vec(re2rh,[0,0,-1]',e2h_atease_rate);
neck2ls_mr = rotate_vec(neck2ls,[0,+1,0]',neck2s_atease_rate);
ls2le_mr = rotate_vec(ls2le,[0,0,-1]',s2e_atease_rate);
le2lh_mr = rotate_vec(le2lh,[0,0,-1]',e2h_atease_rate);

% Result
p_neck_mr = p_root + root2neck_lengthen_rate*root2neck_mr;
p_rs_mr = p_neck_mr + neck2s_lengthen_rate*neck2rs_mr;
p_re_mr = p_rs_mr + s2e_lengthen_rate*rs2re_mr;
p_rh_mr = p_re_mr + e2h_lengthen_rate*re2rh_mr;
p_ls_mr = p_neck_mr + neck2s_lengthen_rate*neck2ls_mr;
p_le_mr = p_ls_mr + s2e_lengthen_rate*ls2le_mr;
p_lh_mr = p_le_mr + e2h_lengthen_rate*le2lh_mr;
