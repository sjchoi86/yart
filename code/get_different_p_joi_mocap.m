function [pm_root,pm_rs,pm_re,pm_rh,pm_ls,pm_le,pm_lh,pm_neck] = ...
    get_different_p_joi_mocap(chain_mocap,joi_mocap)
%
% Get different positions of JOI of MoCap skeleton
%

pm_root = get_p_joi_type(chain_mocap,joi_mocap,'root');
pm_rs = get_p_joi_type(chain_mocap,joi_mocap,'rs');
pm_re = get_p_joi_type(chain_mocap,joi_mocap,'re');
pm_rh = get_p_joi_type(chain_mocap,joi_mocap,'rh');
pm_ls = get_p_joi_type(chain_mocap,joi_mocap,'ls');
pm_le = get_p_joi_type(chain_mocap,joi_mocap,'le');
pm_lh = get_p_joi_type(chain_mocap,joi_mocap,'lh');

% pm_neck = get_p_joi_type(chain_mocap,joi_mocap,'neck');
pm_neck = (pm_rs + pm_ls)/2;
