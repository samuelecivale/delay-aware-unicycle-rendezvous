function [time, P, diagnostics] = simulate_switching_geometric_consensus_2d(P0, radius, tau, T, dt, gain)
%SIMULATE_SWITCHING_GEOMETRIC_CONSENSUS_2D Graph is recomputed from positions.
if nargin < 6, gain = 1.0; end
P0 = double(P0);
time = (0:dt:T)'; K = length(time); N = size(P0,1);
P = zeros(K,N,2); P(1,:,:) = reshape(P0, [1, N, 2]);
delay_steps = max(0, round(tau / dt));
lambda2_hist = zeros(K,1);
edge_hist = zeros(K,1);
for k = 1:K-1
    Pcur = squeeze(P(k,:,:));
    A_k = adjacency_from_positions(Pcur, radius, 1.0);
    L_k = laplacian_matrix(A_k);
    info_k = graph_spectral_info(L_k);
    lambda2_hist(k) = info_k.lambda_2;
    edge_hist(k) = sum(A_k(:)) / 2;
    idx = max(1, k - delay_steps);
    Pdel = squeeze(P(idx,:,:));
    dP = -gain * (L_k * Pdel);
    P_next = Pcur + dt * dP;
    P(k+1,:,:) = reshape(P_next, [1, N, 2]);
end
A_last = adjacency_from_positions(squeeze(P(end,:,:)), radius, 1.0);
info_last = graph_spectral_info(laplacian_matrix(A_last));
lambda2_hist(end) = info_last.lambda_2;
edge_hist(end) = sum(A_last(:)) / 2;
diagnostics = struct('lambda2', lambda2_hist, 'edge_count', edge_hist, 'A_final', A_last);
end
