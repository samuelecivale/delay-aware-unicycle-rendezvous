function [time, P, tau_history] = simulate_time_varying_delay_consensus_2d(L, P0, tau_function, T, dt, gain)
%SIMULATE_TIME_VARYING_DELAY_CONSENSUS_2D Full-state delay with scalar tau(t).
if nargin < 6, gain = 1.0; end
P0 = double(P0); L = double(L);
time = (0:dt:T)'; K = length(time); N = size(P0,1);
P = zeros(K,N,2); P(1,:,:) = reshape(P0, [1, N, 2]);
tau_history = zeros(K,1);
for k = 1:K-1
    tau_k = max(0, tau_function(time(k)));
    tau_history(k) = tau_k;
    idx = max(1, k - round(tau_k / dt));
    Pdel = squeeze(P(idx,:,:));
    dP = -gain * (L * Pdel);
    P_next = squeeze(P(k,:,:)) + dt * dP;
    P(k+1,:,:) = reshape(P_next, [1, N, 2]);
end
if K > 1
    tau_history(end) = tau_history(K-1);
end
end
