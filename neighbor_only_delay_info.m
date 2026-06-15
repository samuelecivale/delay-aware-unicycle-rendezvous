function info = neighbor_only_delay_info(A)
%NEIGHBOR_ONLY_DELAY_INFO Theoretical note for xdot = -D x(t) + A x(t-tau).
info = struct();
info.tau_crit = Inf;
info.is_delay_independent_stable = is_connected_graph(A);
info.reason = 'For connected undirected graphs with nonnegative weights, the disagreement dynamics are delay-independent stable in the ideal continuous neighbor-only model.';
end
