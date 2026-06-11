function [time, X] = simulate_delayed_consensus_1d(L, x0, tau, T, dt, gain)
%SIMULATE_DELAYED_CONSENSUS_1D Simulate xdot(t) = -gain * L * x(t - tau).
%
% Delay is implemented with a fixed-step Euler method.

    if nargin < 6
        gain = 1.0;
    end

    x0 = x0(:);
    N = length(x0);

    time = 0:dt:T;
    K = length(time);

    X = zeros(K, N);
    X(1, :) = x0';

    delay_steps = round(tau / dt);

    for k = 1:(K-1)
        delayed_index = max(1, k - delay_steps);
        x_delay = X(delayed_index, :)';

        dx = -gain * (L * x_delay);

        X(k+1, :) = X(k, :) + dt * dx';
    end
end
