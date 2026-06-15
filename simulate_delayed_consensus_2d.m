function [time, P] = simulate_delayed_consensus_2d(L, P0, tau, T, dt, gain)
%SIMULATE_DELAYED_CONSENSUS_2D Full-state delayed consensus in 2D.
if nargin < 6, gain = 1.0; end
L = double(L);
P0 = double(P0);
time = (0:dt:T)';
K = length(time);
N = size(P0,1);
P = zeros(K, N, 2);
P(1,:,:) = reshape(P0, [1, N, 2]);
delay_steps = max(0, round(tau / dt));
for k = 1:K-1
    idx = max(1, k - delay_steps);
    Pdel = squeeze(P(idx,:,:));
    dP = -gain * (L * Pdel);
    P_next = squeeze(P(k,:,:)) + dt * dP;
    P(k+1,:,:) = reshape(P_next, [1, N, 2]);
end
end
