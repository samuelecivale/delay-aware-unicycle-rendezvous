function info = print_graph_info(A, name)
%PRINT_GRAPH_INFO Print weighted graph information.

    L = laplacian_matrix(A);
    info = graph_spectral_info(L);

    fprintf('===== %s =====\n', name);
    disp('Adjacency A:');
    disp(A);
    disp('Weighted degrees:');
    disp(sum(A, 2)');
    disp('Laplacian eigenvalues:');
    disp(info.eigvals');
    fprintf('lambda_2: %.12g\n', info.lambda_2);
    fprintf('lambda_max: %.12g\n', info.lambda_max);
    fprintf('tau_crit: %.12g\n', info.tau_crit);
    fprintf('connected: %d\n\n', info.is_connected);
end
