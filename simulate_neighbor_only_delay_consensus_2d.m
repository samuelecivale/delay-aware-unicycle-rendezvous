function [time, P] = simulate_neighbor_only_delay_consensus_2d(A, P0, tau, T, dt, gain)
%SIMULATE_NEIGHBOR_ONLY_DELAY_CONSENSUS_2D xdot = -gain D x(t) + gain A x(t-tau).
if nargin < 6, gain = 1.0; end
A = double(A);
D = weighted_degree_matrix(A);
P0 = double(P0);
time = (0:dt:T)';
K = length(time);
N = size(P0,1);
P = zeros(K, N, 2);
P(1,:,:) = reshape(P0, [1, N, 2]);
delay_steps = max(0, round(tau / dt));
for k = 1:K-1
    idx = max(1, k - delay_steps);
    Pcur = squeeze(P(k,:,:));
    Pdel = squeeze(P(idx,:,:));
    dP = gain * (-D * Pcur + A * Pdel);
    P_next = Pcur + dt * dP;
    P(k+1,:,:) = reshape(P_next, [1, N, 2]);
end
end
