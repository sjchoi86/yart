function [pr_root,pr_rs,pr_re,pr_rh,pr_ls,pr_le,pr_lh,pr_neck] = ...
    get_different_p_joi_robot_model(chain_model,joi_model)
%
% Get different positions of JOI of the Robot Model 
%

pr_root = get_p_joi_type(chain_model,joi_model,'root');
pr_rs = get_p_joi_type(chain_model,joi_model,'rs');
pr_re = get_p_joi_type(chain_model,joi_model,'re');
pr_rh = get_p_joi_type(chain_model,joi_model,'rh');
pr_ls = get_p_joi_type(chain_model,joi_model,'ls');
pr_le = get_p_joi_type(chain_model,joi_model,'le');
pr_lh = get_p_joi_type(chain_model,joi_model,'lh');
pr_neck = (pr_rs + pr_ls)/2;
