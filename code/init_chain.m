function chain = init_chain(name)
%
% Initialize kinematic chain
%

chain.name = name; % name 
chain.n_joint = 0;
chain.n_revolute_joint = 0; % no revolute here
chain.joint_names = {};
