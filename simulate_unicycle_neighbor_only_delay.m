function [time, states, commands] = simulate_unicycle_neighbor_only_delay(A, initial_states, tau, T, dt, gain, k_v, k_theta, use_saturation, v_max, omega_max, allow_backward_motion)
%SIMULATE_UNICYCLE_NEIGHBOR_ONLY_DELAY Kinematic unicycle with neighbor-only delayed virtual velocity.
if nargin < 12, allow_backward_motion = true; end
if nargin < 11, omega_max = 4.0; end
if nargin < 10, v_max = 1.5; end
if nargin < 9, use_saturation = true; end
if nargin < 8, k_theta = 3.0; end
if nargin < 7, k_v = 1.0; end
if nargin < 6, gain = 1.0; end
A = double(A);
D = weighted_degree_matrix(A);
states0 = double(initial_states);
time = (0:dt:T)';
K = length(time);
N = size(states0,1);
states = zeros(K, N, 3);
states(1,:,:) = reshape(states0, [1, N, 3]);
commands = zeros(K, N, 2);
delay_steps = max(0, round(tau / dt));
for k = 1:K-1
    idx = max(1, k - delay_steps);
    Pcur = squeeze(states(k,:,1:2));
    Pdel = squeeze(states(idx,:,1:2));
    U = gain * (-D * Pcur + A * Pdel);
    for i = 1:N
        x = states(k,i,1); y = states(k,i,2); theta = states(k,i,3);
        ux = U(i,1); uy = U(i,2);
        speed_des = norm([ux, uy]);
        if speed_des < 1e-10
            e_theta = 0;
        else
            theta_des = atan2(uy, ux);
            e_theta = wrap_to_pi_custom(theta_des - theta);
        end
        v = k_v * speed_des * cos(e_theta);
        omega = k_theta * e_theta;
        if ~allow_backward_motion
            v = max(0, v);
        end
        if use_saturation
            v = min(max(v, -v_max), v_max);
            omega = min(max(omega, -omega_max), omega_max);
        end
        commands(k,i,1) = v;
        commands(k,i,2) = omega;
        states(k+1,i,1) = x + dt * v * cos(theta);
        states(k+1,i,2) = y + dt * v * sin(theta);
        states(k+1,i,3) = wrap_to_pi_custom(theta + dt * omega);
    end
end
if K > 1
    commands(end,:,:) = commands(K-1,:,:);
end
end
