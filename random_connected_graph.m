function A = random_connected_graph(N, p, max_tries, seed)
%RANDOM_CONNECTED_GRAPH Generate an Erdos-Renyi connected undirected graph.
%
% Each possible edge is included with probability p.
% The function retries until a connected graph is found.

    if nargin >= 4 && ~isempty(seed)
        rng(seed);
    end

    for attempt = 1:max_tries
        A = zeros(N, N);

        for i = 1:N
            for j = (i+1):N
                if rand() < p
                    A(i, j) = 1;
                    A(j, i) = 1;
                end
            end
        end

        if is_connected_graph(A)
            return;
        end
    end

    error('Could not generate a connected graph. Try increasing p.');
end
