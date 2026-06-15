function A = weighted_ring_graph(N, weights)
%WEIGHTED_RING_GRAPH Build a ring with one positive weight per edge.
if length(weights) ~= N
    error('weighted_ring_graph:Input', 'weights must have length N.');
end
A = zeros(N,N);
for i = 1:N
    j = mod(i, N) + 1;
    w = weights(i);
    A(i,j) = w;
    A(j,i) = w;
end
end
