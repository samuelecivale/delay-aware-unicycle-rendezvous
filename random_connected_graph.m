function A = random_connected_graph(N, p_edge, max_tries, seed)
%RANDOM_CONNECTED_GRAPH Generate an Erdos-Renyi undirected connected graph.
if nargin >= 4 && ~isempty(seed)
    rng(seed);
end
for attempt = 1:max_tries
    R = rand(N,N);
    A = double(triu(R < p_edge, 1));
    A = A + A';
    if is_connected_graph(A)
        return;
    end
end
% Fallback: ring graph is connected.
A = adjacency_matrix('ring', N);
warning('random_connected_graph:Fallback', 'Could not sample connected graph; returned ring.');
end
