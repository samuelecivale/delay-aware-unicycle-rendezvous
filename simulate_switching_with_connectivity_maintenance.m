function [time, P, diagnostics] = simulate_switching_with_connectivity_maintenance(P0, radius, tau, T, dt, gain, use_maintenance, lambda2_min, margin, k_conn, external_disturbance, u_max)
%SIMULATE_SWITCHING_WITH_CONNECTIVITY_MAINTENANCE Switching graph with lambda_2-based heuristic maintenance.
if nargin < 12, u_max = 2.5; end
if nargin < 11, external_disturbance = []; end
if nargin < 10, k_conn = 2.5; end
if nargin < 9, margin = 0.8; end
if nargin < 8, lambda2_min = 0.10; end
if nargin < 7, use_maintenance = true; end
if nargin < 6, gain = 1.0; end
P0 = double(P0);
time = (0:dt:T)'; K = length(time); N = size(P0,1);
P = zeros(K,N,2); P(1,:,:) = reshape(P0, [1, N, 2]);
delay_steps = max(0, round(tau / dt));
lambda2_hist = zeros(K,1); edge_hist = zeros(K,1);
for k = 1:K-1
    Pcur = squeeze(P(k,:,:));
    A_k = adjacency_from_positions(Pcur, radius, 1.0);
    L_k = laplacian_matrix(A_k);
    info_k = graph_spectral_info(L_k);
    lambda2_hist(k) = info_k.lambda_2;
    edge_hist(k) = sum(A_k(:)) / 2;
    idx = max(1, k - delay_steps);
    Pdel = squeeze(P(idx,:,:));
    U = -gain * (L_k * Pdel);
    if use_maintenance && lambda2_hist(k) < lambda2_min
        for i = 1:N
            for j = i+1:N
                diff = Pcur(j,:) - Pcur(i,:);
                dist = norm(diff) + 1e-9;
                if dist >= radius - margin && dist <= radius + margin
                    direction = diff / dist;
                    normalized = max(0.0, dist - (radius - margin)) / margin;
                    force = k_conn * normalized * direction;
                    U(i,:) = U(i,:) + force;
                    U(j,:) = U(j,:) - force;
                end
            end
        end
    end
    if ~isempty(external_disturbance)
        U = U + external_disturbance(Pcur, time(k));
    end
    for i = 1:N
        U(i,:) = clip_vector_norm(U(i,:), u_max);
    end
    P_next = Pcur + dt * U;
    P(k+1,:,:) = reshape(P_next, [1, N, 2]);
end
A_last = adjacency_from_positions(squeeze(P(end,:,:)), radius, 1.0);
info_last = graph_spectral_info(laplacian_matrix(A_last));
lambda2_hist(end) = info_last.lambda_2;
edge_hist(end) = sum(A_last(:)) / 2;
diagnostics = struct('lambda2', lambda2_hist, 'edge_count', edge_hist, 'A_final', A_last);
end
