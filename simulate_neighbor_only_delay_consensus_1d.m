function [time, X] = simulate_neighbor_only_delay_consensus_1d(A, x0, tau, T, dt, gain)
%SIMULATE_NEIGHBOR_ONLY_DELAY_CONSENSUS_1D
%
% Simulate:
%   xdot(t) = -gain * D * x(t) + gain * A * x(t - tau)
%
% This models an agent that knows its current state but receives delayed
% neighbor states.

    if nargin < 6
        gain = 1.0;
    end

    A = double(A);
    D = weighted_degree_matrix(A);

    x0 = x0(:);
    N = length(x0);

    time = 0:dt:T;
    K = length(time);

    X = zeros(K, N);
    X(1, :) = x0';

    delay_steps = round(tau / dt);

    for k = 1:(K-1)
        delayed_index = max(1, k - delay_steps);

        x_current = X(k, :)';
        x_delay = X(delayed_index, :)';

        dx = gain * (-D * x_current + A * x_delay);

        X(k+1, :) = X(k, :) + dt * dx';
    end
end
