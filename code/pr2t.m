function T = pr2t(p,R)
%
% Get T from p and R
%

if nargin == 1
    if numel(p) == 3
        % p is given
        R = eye(3,3);
    elseif numel(p) == 3
        % R is given at 'p'
        R = p;
        p = [0,0,0];
    end
end

T = [R,reshape(p,[3,1]);...
    0,0,0,1];
