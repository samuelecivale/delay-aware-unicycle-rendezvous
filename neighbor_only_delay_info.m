function info = neighbor_only_delay_info(A)
%NEIGHBOR_ONLY_DELAY_INFO Stability information for the neighbor-only delay model.
%
% Model:
%   xdot(t) = -D x(t) + A x(t - tau)
%
% For an undirected connected graph with nonnegative weights and positive
% degrees, this model is delay-independent stable in the disagreement
% subspace. Therefore there is no finite tau_crit analogous to
% pi/(2*lambda_max(L)). In the ideal continuous-time model:
%
%   tau_crit_neighbor_only = Inf
%
% The zero root corresponds to the consensus mode. If the graph is
% disconnected, global consensus is not guaranteed.

    L = laplacian_matrix(A);
    graph_info = graph_spectral_info(L);

    info.is_connected = graph_info.is_connected;
    info.zero_count = graph_info.zero_count;
    info.tau_crit_neighbor_only = Inf;
    info.note = 'Delay-independent stable for connected undirected graphs with nonnegative weights; no finite tau_crit.';

    D = weighted_degree_matrix(A);

    % For regular graphs, we can also expose the scalar modal quantities:
    % D = d I and A has eigenvalues alpha_k.
    degrees = diag(D);
    if max(abs(degrees - degrees(1))) < 1e-10
        d = degrees(1);
        alpha = sort(real(eig(A)), 'descend');
        info.is_regular = true;
        info.regular_degree = d;
        info.adjacency_eigvals = alpha;
        info.scalar_condition = '|alpha_k| <= d; disagreement modes are delay-independent stable.';
    else
        info.is_regular = false;
        info.regular_degree = NaN;
        info.adjacency_eigvals = sort(real(eig(A)), 'descend');
        info.scalar_condition = 'Non-regular graph: use general delay-independent consensus result or characteristic roots.';
    end
end
