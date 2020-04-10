function R = rpy2r(rpy_deg)
% 
% Euler angle (roll, pitch, and yaw) in degree to rotation matrix
%

r_deg = rpy_deg(1);
p_deg = rpy_deg(2);
y_deg = rpy_deg(3);

D2R = pi/180;
r_rad = r_deg*D2R;
p_rad = p_deg*D2R;
y_rad = y_deg*D2R;

cos_r = cos(r_rad); sin_r = sin(r_rad);
cos_p = cos(p_rad); sin_p = sin(p_rad);
cos_y = cos(y_rad); sin_y = sin(y_rad);

R = [cos_y*cos_p, -sin_y*cos_r+cos_y*sin_p*sin_r, sin_y*sin_r+cos_y*sin_p*cos_r ; ...
    sin_y*cos_p, cos_y*cos_r+sin_y*sin_p*sin_r, -cos_y*sin_r+sin_y*sin_p*cos_r ; ...
    -sin_p, cos_p*sin_r, cos_p*cos_r];
