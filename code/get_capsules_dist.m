function [dist,cline1,cline2] ...
    = get_capsules_dist(cap_opt1,p1,R1,cap_opt2,p2,R2,inc_rate)

if nargin == 6
    inc_rate = 1;
end

cline1 = get_line_capsule(p1,R1,cap_opt1);
cline2 = get_line_capsule(p2,R2,cap_opt2);

if inc_rate ~= 1
    mid1 = 0.5*(cline1.e1+cline1.e2);
    e2toe1 = (cline1.e1-cline1.e2);
    cline1.e1 = mid1 + e2toe1*0.5*inc_rate;
    cline1.e2 = mid1 - e2toe1*0.5*inc_rate;
    cap_opt1.radius = cap_opt1.radius * inc_rate;
    
    mid2 = 0.5*(cline2.e1+cline2.e2);
    e2toe1 = (cline2.e1-cline2.e2);
    cline2.e1 = mid2 + e2toe1*0.5*inc_rate;
    cline2.e2 = mid2 - e2toe1*0.5*inc_rate;
    cap_opt2.radius = cap_opt2.radius * inc_rate;
end

% Actual distance
dist = get_dist_lines(cline1.e1', cline1.e2', cline2.e1', cline2.e2')...
    -(cap_opt1.radius + cap_opt2.radius);
