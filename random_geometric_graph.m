function [A, pos] = random_geometric_graph(N, radius, box_size, max_tries, seed)
%RANDOM_GEOMETRIC_GRAPH Generate a connected random geometric graph.
if nargin >= 5 && ~isempty(seed)
    rng(seed);
end
for attempt = 1:max_tries
    pos = box_size * (rand(N,2) - 0.5);
    A = adjacency_from_positions(pos, radius, 1.0);
    if is_connected_graph(A)
        return;
    end
end
% Fallback: deterministic ring-like geometry.
theta = linspace(0, 2*pi, N+1)'; theta(end) = [];
pos = [cos(theta), sin(theta)] * min(radius * 0.45, box_size * 0.35);
A = adjacency_matrix('ring', N);
warning('random_geometric_graph:Fallback', 'Could not sample connected geometric graph; returned ring-like fallback.');
end
