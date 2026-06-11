function connected = is_connected_graph(A)
%IS_CONNECTED_GRAPH Check connectivity through Laplacian eigenvalues.
    L = laplacian_matrix(A);
    info = graph_spectral_info(L);
    connected = info.is_connected;
end
