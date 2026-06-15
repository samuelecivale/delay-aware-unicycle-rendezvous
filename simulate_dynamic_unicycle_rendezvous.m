function [time, states, commands] = simulate_dynamic_unicycle_rendezvous(L, initial_states, tau, T, dt, gain, k_v, k_theta, k_omega, v_max, omega_max, a_max, alpha_max, allow_backward_motion)
%SIMULATE_DYNAMIC_UNICYCLE_RENDEZVOUS Dynamic unicycle with vdot=a and omegadot=alpha.
if nargin < 14, allow_backward_motion = true; end
if nargin < 13, alpha_max = 6.0; end
if nargin < 12, a_max = 1.2; end
if nargin < 11, omega_max = 4.0; end
if nargin < 10, v_max = 1.5; end
if nargin < 9, k_omega = 4.0; end
if nargin < 8, k_theta = 3.0; end
if nargin < 7, k_v = 2.0; end
if nargin < 6, gain = 1.0; end
L = double(L); initial_states = double(initial_states);
time = (0:dt:T)'; K = length(time); N = size(initial_states,1);
states = zeros(K,N,5); states(1,:,1:3) = reshape(initial_states, [1, N, 3]);
commands = zeros(K,N,2);
delay_steps = max(0, round(tau / dt));
for k = 1:K-1
    idx = max(1, k - delay_steps);
    Pdel = squeeze(states(idx,:,1:2));
    U = -gain * (L * Pdel);
    for i = 1:N
        x = states(k,i,1); y = states(k,i,2); theta = states(k,i,3);
        v_i = states(k,i,4); omega_i = states(k,i,5);
        ux = U(i,1); uy = U(i,2);
        speed_des = norm([ux, uy]);
        if speed_des < 1e-10
            e_theta = 0;
        else
            theta_des = atan2(uy, ux);
            e_theta = wrap_to_pi_custom(theta_des - theta);
        end
        v_des = speed_des * cos(e_theta);
        if ~allow_backward_motion, v_des = max(0, v_des); end
        omega_des = k_theta * e_theta;
        a = min(max(k_v * (v_des - v_i), -a_max), a_max);
        alpha = min(max(k_omega * (omega_des - omega_i), -alpha_max), alpha_max);
        v_next = min(max(v_i + dt * a, -v_max), v_max);
        omega_next = min(max(omega_i + dt * alpha, -omega_max), omega_max);
        theta_next = wrap_to_pi_custom(theta + dt * omega_next);
        states(k+1,i,1) = x + dt * v_next * cos(theta);
        states(k+1,i,2) = y + dt * v_next * sin(theta);
        states(k+1,i,3) = theta_next;
        states(k+1,i,4) = v_next;
        states(k+1,i,5) = omega_next;
        commands(k,i,1) = a;
        commands(k,i,2) = alpha;
    end
end
if K > 1
    commands(end,:,:) = commands(K-1,:,:);
end
end
