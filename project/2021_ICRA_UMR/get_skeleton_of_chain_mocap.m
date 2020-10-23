function x_skeleton = get_skeleton_of_chain_mocap(chain_mocap,joi_mocap)
%
% Get skeleton representation of the robot model
%
% Skeleton parametrization (21-dimensional vector):
% : unit vectors of [h2n,n2rs,rs2re,re2rh,n2ls,ls2le,le2lh]
%
% This representation is translation-invariant 
%

[p_root,p_rs,p_re,p_rh,p_ls,p_le,p_lh,p_neck] = ...
    get_different_p_joi_mocap(chain_mocap,joi_mocap);

x_skeleton = [...
    get_uv(p_neck-p_root); ... % root to neck 
    get_uv(p_rs-p_neck); ... % neck to right shoulder
    get_uv(p_re-p_rs); ... % right shoulder to right elbow
    get_uv(p_rh-p_re); ... % right elbow to right hand
    get_uv(p_ls-p_neck); ... % neck to left shoulder
    get_uv(p_le-p_ls); ... % left shoulder to left elbow
    get_uv(p_lh-p_le); ... % left elbow to left hand 
    ]';
