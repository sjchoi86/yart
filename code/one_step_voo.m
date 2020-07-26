function voo = one_step_voo(voo)

voo.tick = voo.tick + 1; % increase tick

if (rand < voo.omega) || (voo.tick <= 3) % explorataion
    x = voo.px(1);
else % exploitation
    f_list = voo.f(voo.x_list(1:(voo.tick-1),:)); % f values so far
    [~,max_idx] = max(f_list); % get current best index
    x_star = voo.x_list(max_idx,:); % current best input
    x_list_temp = voo.x_list(1:(voo.tick-1),:);
    x_list_temp(max_idx,:) = inf*[1,1];
    d_min = min(vecnorm(x_list_temp-x_star,2,2));
    exploit_idx = 0;
    d_temp_list = inf*ones(1,voo.max_exploit);
    x_temp_list = zeros(voo.max_exploit,2);
    while 1
        exploit_idx = exploit_idx + 1;
        x_temp = voo.px(1); % new sample
        d_temp = norm(x_temp-x_star); % check distance to current best one
        if d_temp < d_min % rejection sampling
            x = x_temp;
            break;
        end
        d_temp_list(exploit_idx) = d_temp;
        x_temp_list(exploit_idx,:) = x_temp;
        if exploit_idx >= voo.max_exploit
            [~,min_idx] = min(d_temp_list);
            x = x_temp_list(min_idx,:);
            break;
        end
    end
end
% Append
voo.x_list(voo.tick,:) = x;
