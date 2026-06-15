function info = graph_spectral_info(L)
%GRAPH_SPECTRAL_INFO Return spectral quantities of a graph Laplacian.
L = double(L);
eigvals = sort(real(eig(L)));
N = length(eigvals);
zero_tol = 1e-8;
info = struct();
info.eigvals = eigvals;
info.zero_count = sum(abs(eigvals) < zero_tol);
if N >= 2
    info.lambda_2 = eigvals(2);
else
    info.lambda_2 = 0;
end
if N >= 1
    info.lambda_max = eigvals(end);
else
    info.lambda_max = 0;
end
if info.lambda_max > zero_tol
    info.tau_crit = pi / (2 * info.lambda_max);
else
    info.tau_crit = Inf;
end
info.is_connected = (info.zero_count == 1 && info.lambda_2 > zero_tol);
end
