function [time, P] = simulate_delayed_consensus_2d(L, P0, tau, T, dt, gain)
%SIMULATE_DELAYED_CONSENSUS_2D Simulate Pdot(t) = -gain * L * P(t - tau).
%
% P is K x N x 2.

    if nargin < 6
        gain = 1.0;
    end

    N = size(P0, 1);

    time = 0:dt:T;
    K = length(time);

    P = zeros(K, N, 2);
    P(1, :, :) = P0;

    delay_steps = round(tau / dt);

    for k = 1:(K-1)
        delayed_index = max(1, k - delay_steps);

        P_delay = squeeze(P(delayed_index, :, :));
        dP = -gain * (L * P_delay);

        P(k+1, :, :) = squeeze(P(k, :, :)) + dt * dP;
    end
end
