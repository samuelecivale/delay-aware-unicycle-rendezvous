function print_graph_info(A, name)
%PRINT_GRAPH_INFO Print adjacency, Laplacian and spectral quantities.
if nargin < 2
    name = 'graph';
end
L = laplacian_matrix(A);
info = graph_spectral_info(L);
fprintf('\n--- %s ---\n', name);
fprintf('Number of nodes: %d\n', size(A,1));
fprintf('Number of edges: %.0f\n', sum(A(:) > 0) / 2);
fprintf('lambda_2: %.12g\n', info.lambda_2);
fprintf('lambda_max: %.12g\n', info.lambda_max);
fprintf('tau_crit: %.12g\n', info.tau_crit);
fprintf('connected: %d\n', info.is_connected);
end
