function A = weighted_ring_graph(N, weights)
%WEIGHTED_RING_GRAPH Create a weighted ring graph.
%
% weights(i) is the weight of edge i -- i+1, with N connected to 1.

    if length(weights) ~= N
        error('weights must have length N.');
    end

    A = zeros(N, N);

    for i = 1:N
        j = mod(i, N) + 1;
        A(i, j) = weights(i);
        A(j, i) = weights(i);
    end
end
