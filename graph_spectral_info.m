function info = graph_spectral_info(L)
%GRAPH_SPECTRAL_INFO Return eigenvalues and spectral graph quantities.
%
% For an undirected graph, L is symmetric and has real nonnegative eigenvalues.

    tol = 1e-9;

    eigvals = sort(real(eig(L)));
    zero_count = sum(abs(eigvals) < tol);
    is_connected = (zero_count == 1);

    if length(eigvals) >= 2
        lambda_2 = eigvals(2);
    else
        lambda_2 = NaN;
    end

    lambda_max = eigvals(end);

    if lambda_max > tol
        tau_crit = pi / (2 * lambda_max);
    else
        tau_crit = Inf;
    end

    info.eigvals = eigvals;
    info.zero_count = zero_count;
    info.is_connected = is_connected;
    info.lambda_2 = lambda_2;
    info.lambda_max = lambda_max;
    info.tau_crit = tau_crit;
end
