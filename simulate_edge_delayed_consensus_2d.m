function [time, P] = simulate_edge_delayed_consensus_2d(A, P0, Tau, T, dt, gain)
%SIMULATE_EDGE_DELAYED_CONSENSUS_2D Different delay tau_ij on each edge.
if nargin < 6, gain = 1.0; end
A = double(A); P0 = double(P0); Tau = double(Tau);
time = (0:dt:T)';
K = length(time); N = size(P0,1);
P = zeros(K,N,2); P(1,:,:) = reshape(P0, [1, N, 2]);
delay_steps = max(0, round(Tau / dt));
for k = 1:K-1
    dP = zeros(N,2);
    for i = 1:N
        for j = 1:N
            if i ~= j && A(i,j) > 0
                idx = max(1, k - delay_steps(i,j));
                Pi = squeeze(P(idx,i,:))';
                Pj = squeeze(P(idx,j,:))';
                dP(i,:) = dP(i,:) - gain * A(i,j) * (Pi - Pj);
            end
        end
    end
    P_next = squeeze(P(k,:,:)) + dt * dP;
    P(k+1,:,:) = reshape(P_next, [1, N, 2]);
end
end
