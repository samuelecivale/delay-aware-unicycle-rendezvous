function [time, states, commands, diagnostics] = simulate_unicycle_with_avoidance(L, initial_states, tau, T, dt, gain, k_v, k_theta, use_saturation, v_max, omega_max, allow_backward_motion, collision_avoidance, d_safe, k_rep, obstacles, k_obs, obstacle_influence, u_max)
%SIMULATE_UNICYCLE_WITH_AVOIDANCE Kinematic unicycle with collision/obstacle repulsion.
if nargin < 19, u_max = 3.0; end
if nargin < 18, obstacle_influence = 1.8; end
if nargin < 17, k_obs = 0.15; end
if nargin < 16, obstacles = []; end
if nargin < 15, k_rep = 0.25; end
if nargin < 14, d_safe = 0.7; end
if nargin < 13, collision_avoidance = false; end
if nargin < 12, allow_backward_motion = true; end
if nargin < 11, omega_max = 4.0; end
if nargin < 10, v_max = 1.5; end
if nargin < 9, use_saturation = true; end
if nargin < 8, k_theta = 3.0; end
if nargin < 7, k_v = 1.0; end
if nargin < 6, gain = 1.0; end
L = double(L); initial_states = double(initial_states);
time = (0:dt:T)'; K = length(time); N = size(initial_states,1);
states = zeros(K,N,3); states(1,:,:) = reshape(initial_states, [1, N, 3]);
commands = zeros(K,N,2);
delay_steps = max(0, round(tau / dt));
min_pair = zeros(K,1); min_obs = zeros(K,1);
for k = 1:K-1
    idx = max(1, k - delay_steps);
    Pcur = squeeze(states(k,:,1:2));
    Pdel = squeeze(states(idx,:,1:2));
    U = -gain * (L * Pdel);
    if collision_avoidance
        for i = 1:N
            for j = i+1:N
                diff = Pcur(i,:) - Pcur(j,:);
                dist = norm(diff) + 1e-9;
                if dist < d_safe
                    direction = diff / dist;
                    strength = k_rep * (1.0 / dist - 1.0 / d_safe) / (dist^2);
                    force = strength * direction;
                    U(i,:) = U(i,:) + force;
                    U(j,:) = U(j,:) - force;
                end
            end
        end
    end
    if ~isempty(obstacles)
        for i = 1:N
            for o = 1:length(obstacles)
                c = obstacles(o).center(:)';
                r = obstacles(o).radius;
                influence = obstacle_influence;
                if isfield(obstacles(o), 'influence')
                    influence = obstacles(o).influence;
                end
                diff = Pcur(i,:) - c;
                center_dist = norm(diff) + 1e-9;
                clearance = center_dist - r;
                if clearance < influence
                    direction = diff / center_dist;
                    effective = max(clearance, 1e-3);
                    strength = k_obs * (1.0 / effective - 1.0 / influence) / (effective^2);
                    U(i,:) = U(i,:) + strength * direction;
                end
            end
        end
    end
    for i = 1:N
        U(i,:) = clip_vector_norm(U(i,:), u_max);
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
        if ~allow_backward_motion, v = max(0, v); end
        if use_saturation
            v = min(max(v, -v_max), v_max);
            omega = min(max(omega, -omega_max), omega_max);
        end
        commands(k,i,1) = v; commands(k,i,2) = omega;
        states(k+1,i,1) = x + dt * v * cos(theta);
        states(k+1,i,2) = y + dt * v * sin(theta);
        states(k+1,i,3) = wrap_to_pi_custom(theta + dt * omega);
    end
    min_pair(k) = pairwise_min_distance(Pcur);
    min_obs(k) = min_obstacle_clearance(Pcur, obstacles);
end
if K > 1
    commands(end,:,:) = commands(K-1,:,:);
end
min_pair(end) = pairwise_min_distance(squeeze(states(end,:,1:2)));
min_obs(end) = min_obstacle_clearance(squeeze(states(end,:,1:2)), obstacles);
diagnostics = struct('min_pair_distance', min_pair, 'min_obstacle_clearance', min_obs);
end
