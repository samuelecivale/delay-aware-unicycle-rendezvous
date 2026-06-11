function [A, positions] = random_geometric_graph(N, radius, domain_size, max_tries, seed)
%RANDOM_GEOMETRIC_GRAPH Generate a connected random geometric graph.
%
% Agents are placed randomly in a square.
% Two agents are connected if their distance is <= radius.

    if nargin >= 5 && ~isempty(seed)
        rng(seed);
    end

    for attempt = 1:max_tries
        positions = -domain_size/2 + domain_size * rand(N, 2);
        A = zeros(N, N);

        for i = 1:N
            for j = (i+1):N
                dist = norm(positions(i,:) - positions(j,:));
                if dist <= radius
                    A(i, j) = 1;
                    A(j, i) = 1;
                end
            end
        end

        if is_connected_graph(A)
            return;
        end
    end

    error('Could not generate a connected geometric graph. Try increasing radius.');
end
