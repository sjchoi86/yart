function rpy_deg = r2rpy(R)
%
% Get roll, pitch, and yaw [in radian] from a rotational matrix
%  alpha: yaw
%  beta: pitch
%  gamma: roll 
%

alpha = atan2(R(2,1),R(1,1));
beta = atan2(-R(3,1),sqrt(R(3,2)*R(3,2)+R(3,3)*R(3,3)));
gamma = atan2(R(3,2),R(3,3));

rpy_deg = [gamma,beta,alpha];
