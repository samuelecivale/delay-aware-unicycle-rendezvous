function [time, X] = simulate_neighbor_only_delay_consensus_1d(A, x0, tau, T, dt, gain)
%SIMULATE_NEIGHBOR_ONLY_DELAY_CONSENSUS_1D xdot = -gain D x(t) + gain A x(t-tau).
if nargin < 6, gain = 1.0; end
A = double(A);
D = weighted_degree_matrix(A);
x0 = double(x0(:));
time = (0:dt:T)';
K = length(time);
N = length(x0);
X = zeros(K, N);
X(1,:) = x0';
delay_steps = max(0, round(tau / dt));
for k = 1:K-1
    idx = max(1, k - delay_steps);
    xcur = X(k,:)';
    xdel = X(idx,:)';
    dX = gain * (-D * xcur + A * xdel);
    X(k+1,:) = X(k,:) + dt * dX';
end
end
