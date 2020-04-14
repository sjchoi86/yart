function ik = add_ik(ik,varargin)
%
% Add a target to the IK structure
%

% Parse options
parser = inputParser;
addParameter(parser,'joint_name','');
addParameter(parser,'p',[0,0,0]');
addParameter(parser,'R',eye(3,3));
addParameter(parser,'IK_P',0);
addParameter(parser,'IK_R',0);
addParameter(parser,'weight',1.0);
parse(parser,varargin{:});
joint_name = parser.Results.joint_name;
p = parser.Results.p;
R = parser.Results.R;
IK_P = parser.Results.IK_P;
IK_R = parser.Results.IK_R;
weight = parser.Results.weight;


% Append IK target information
ik.n_target = ik.n_target + 1;
ik.targets(ik.n_target).joint_name = joint_name;
ik.targets(ik.n_target).joint_idx = idx_cell(ik.chain.joint_names,joint_name); % joint index 
ik.targets(ik.n_target).p = p;
ik.targets(ik.n_target).R = R;
ik.targets(ik.n_target).IK_P = IK_P;
ik.targets(ik.n_target).IK_R = IK_R;
ik.targets(ik.n_target).weight = weight;

