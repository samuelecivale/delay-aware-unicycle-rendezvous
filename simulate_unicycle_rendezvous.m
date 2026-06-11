function [time, states, commands] = simulate_unicycle_rendezvous(...
    L, initial_states, tau, T, dt, gain, k_v, k_theta, ...
    use_saturation, v_max, omega_max, allow_backward_motion)
%SIMULATE_UNICYCLE_RENDEZVOUS
%
% Full-state delayed unicycle rendezvous.
%
% The consensus law produces a desired planar velocity:
%   U(t) = -gain * L * P(t - tau)
%
% Then U_i is converted to unicycle commands:
%   theta_des = atan2(u_y, u_x)
%   e_theta = theta_des - theta_i
%   omega_i = k_theta * e_theta
%   v_i = k_v * ||u_i|| * cos(e_theta)

    if nargin < 12
        allow_backward_motion = true;
    end

    N = size(initial_states, 1);

    time = 0:dt:T;
    K = length(time);

    states = zeros(K, N, 3);
    states(1, :, :) = initial_states;

    commands = zeros(K, N, 2);

    delay_steps = round(tau / dt);

    for k = 1:(K-1)
        delayed_index = max(1, k - delay_steps);

        P_delay = squeeze(states(delayed_index, :, 1:2));
        U = -gain * (L * P_delay);

        for i = 1:N
            x_i = states(k, i, 1);
            y_i = states(k, i, 2);
            theta_i = states(k, i, 3);

            ux = U(i, 1);
            uy = U(i, 2);

            speed_des = norm([ux, uy]);

            if speed_des < 1e-10
                e_theta = 0.0;
            else
                theta_des = atan2(uy, ux);
                e_theta = wrap_to_pi_custom(theta_des - theta_i);
            end

            v = k_v * speed_des * cos(e_theta);
            omega = k_theta * e_theta;

            if ~allow_backward_motion
                v = max(0.0, v);
            end

            if use_saturation
                v = min(max(v, -v_max), v_max);
                omega = min(max(omega, -omega_max), omega_max);
            end

            commands(k, i, 1) = v;
            commands(k, i, 2) = omega;

            states(k+1, i, 1) = x_i + dt * v * cos(theta_i);
            states(k+1, i, 2) = y_i + dt * v * sin(theta_i);
            states(k+1, i, 3) = wrap_to_pi_custom(theta_i + dt * omega);
        end
    end

    commands(K, :, :) = commands(K-1, :, :);
end
