function [time, X] = simulate_delayed_consensus_1d(L, x0, tau, T, dt, gain)
%SIMULATE_DELAYED_CONSENSUS_1D Full-state delayed consensus: xdot = -gain L x(t-tau).
if nargin < 6, gain = 1.0; end
L = double(L);
x0 = double(x0(:));
time = (0:dt:T)';
K = length(time);
N = length(x0);
X = zeros(K, N);
X(1,:) = x0';
delay_steps = max(0, round(tau / dt));
for k = 1:K-1
    idx = max(1, k - delay_steps);
    dX = -gain * (L * X(idx,:)')';
    X(k+1,:) = X(k,:) + dt * dX;
end
end
