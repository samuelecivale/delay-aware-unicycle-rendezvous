function [time, P] = simulate_neighbor_only_delay_consensus_2d(A, P0, tau, T, dt, gain)
%SIMULATE_NEIGHBOR_ONLY_DELAY_CONSENSUS_2D
%
% Simulate:
%   Pdot(t) = -gain * D * P(t) + gain * A * P(t - tau)

    if nargin < 6
        gain = 1.0;
    end

    A = double(A);
    D = weighted_degree_matrix(A);

    N = size(P0, 1);

    time = 0:dt:T;
    K = length(time);

    P = zeros(K, N, 2);
    P(1, :, :) = P0;

    delay_steps = round(tau / dt);

    for k = 1:(K-1)
        delayed_index = max(1, k - delay_steps);

        P_current = squeeze(P(k, :, :));
        P_delay = squeeze(P(delayed_index, :, :));

        dP = gain * (-D * P_current + A * P_delay);

        P(k+1, :, :) = P_current + dt * dP;
    end
end
