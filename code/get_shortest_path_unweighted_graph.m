function [v_trace,n_check] = get_shortest_path_unweighted_graph(B,v_src,v_final)

% Initialize Q and R
n_vertex = length(B);
Q.queue = zeros(1,n_vertex); % queueQ = []; % queue
Q.n = 0;
distances = inf*ones(1,n_vertex); % distances
visited = zeros(1,n_vertex);
parents = zeros(1,n_vertex); % parent vertices

% Frist step
Q = enqueue(Q,v_src); % enqueue initial vertex 
distances(v_src) = 0;
visited(v_src) = 1;

% Iterate
n_check = 0;
while true
    n_check = n_check + 1;
    % Dequeue one vertex
    [Q,v_check] = dequeue(Q); 
    % Check for connected vertices
    v_list = B{v_check};
    for v_idx = 1:length(v_list) % for connected vertices
        v_i = v_list(v_idx);
        if ~visited(v_i) % if not visited
            visited(v_i) = 1; % make it visited
            distances(v_i) = distances(v_check)+1; % dist + 1
            parents(v_i) = v_check;
            Q = enqueue(Q,v_i); % bread first search
        end
    end
    % Terminate
    if v_check == v_final
        break;
    end
end

% Trace back from v_final to v_init
v_check = v_final;
v_trace = v_check;
while v_check ~= v_src
    v_check = parents(v_check);
    v_trace = [v_check,v_trace]; % append to front
end

function Q = enqueue(Q,val)
Q.n = Q.n + 1;
Q.queue(Q.n) = val;

function [Q,val] = dequeue(Q)
val = Q.queue(1); 
Q.n = Q.n - 1;
Q.queue(1:Q.n) = Q.queue(2:(Q.n+1));
