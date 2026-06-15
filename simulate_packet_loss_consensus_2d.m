function [time, P, diagnostics] = simulate_packet_loss_consensus_2d(A, P0, tau, p_loss, T, dt, gain, seed)
%SIMULATE_PACKET_LOSS_CONSENSUS_2D Drop directed messages independently.
if nargin < 8, seed = []; end
if nargin < 7, gain = 1.0; end
if ~isempty(seed), rng(seed); end
A = double(A); P0 = double(P0);
time = (0:dt:T)'; K = length(time); N = size(P0,1);
P = zeros(K,N,2); P(1,:,:) = reshape(P0, [1, N, 2]);
delay_steps = max(0, round(tau / dt));
received = 0; attempted = 0;
for k = 1:K-1
    idx = max(1, k - delay_steps);
    Pdel = squeeze(P(idx,:,:));
    dP = zeros(N,2);
    for i = 1:N
        for j = 1:N
            if i ~= j && A(i,j) > 0
                attempted = attempted + 1;
                if rand() < p_loss
                    continue;
                end
                received = received + 1;
                dP(i,:) = dP(i,:) - gain * A(i,j) * (Pdel(i,:) - Pdel(j,:));
            end
        end
    end
    P_next = squeeze(P(k,:,:)) + dt * dP;
    P(k+1,:,:) = reshape(P_next, [1, N, 2]);
end
if attempted > 0
    rate = received / attempted;
else
    rate = NaN;
end
diagnostics = struct('received_messages', received, 'attempted_messages', attempted, 'empirical_reception_rate', rate);
end
