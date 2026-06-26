function main()
% Control of Multi-Robot Systems - Project 2026
% ============================================================
% ONE MAIN: UNICYCLE-ONLY RENDEZVOUS + TAU_CRIT STUDY
% ============================================================
%
% This file replaces the previous multi-main workflow when the deliverable
% only needs the unicycle extension.  It is intentionally self-contained:
% all helper functions are placed at the end of this same file.
%
% The script studies the same ideas that were previously tested on point-mass
% / single-integrator agents, but the simulated agents are now unicycles:
%
%   xdot_i     = v_i cos(theta_i)
%   ydot_i     = v_i sin(theta_i)
%   thetadot_i = omega_i
%
% The desired Cartesian velocity is produced by a delayed consensus law and
% is then tracked by a unicycle heading controller.
%
% Delay models included:
%   1. full_state     : u(t) = -k L p(t - tau)
%   2. neighbor_only  : u_i(t) = -k sum_j a_ij (p_i(t) - p_j(t - tau))
%
% Important note:
%   tau_crit = pi/(2*k*lambda_max(L)) is the exact linear critical delay for
%   the full-state single-integrator reference model.  In the unicycle case it
%   is used as the reference threshold for the consensus vector field.  The
%   nonlinear heading dynamics, saturation and numerical integration can make
%   the observed behavior slightly different around the boundary.
%
% Outputs:
%   unicycle_only_outputs/figures
%   unicycle_only_outputs/tables
%   unicycle_only_outputs/videos    (only if MAKE_VIDEOS = true)

close all;
clc;

% ============================================================
% Compatibility configuration
% ============================================================
MAKE_FIGURES = true;
CLOSE_FIGURES_AFTER_SAVE = true;
MAKE_VIDEOS = true;        % a video is generated for every test and controller

% ============================================================
% Controller switch
% ============================================================
% 'vector_field' : previous project controller. A delayed Cartesian
%                  consensus vector field is generated first, then tracked
%                  by a unicycle heading controller.
% 'paper'        : paper-inspired unicycle controller. The same delayed
%                  interaction force is projected on the current unicycle
%                  heading, as in the nonholonomic rendezvous/formation
%                  controllers based on potential fields. This is the
%                  recommended/default option if the instructor expects
%                  the controller derived from the reference papers.
CONTROLLER_TYPE = 'paper';
% If true, the whole experiment suite is executed twice: once with the
% paper-inspired controller and once with the vector-field controller.
% Outputs are automatically separated into different folders.
RUN_BOTH_CONTROLLERS = true;


try
    if exist('OCTAVE_VERSION', 'builtin')
        available_toolkits = available_graphics_toolkits();
        if ~isempty(available_toolkits)
            graphics_toolkit(available_toolkits{1});
        end
    end
catch
    fprintf('Warning: could not configure graphics toolkit. Figures may not be saved.\n');
end

try
    set(0, 'DefaultFigureVisible', 'off');
catch
end

safe_rng_local(7);

if RUN_BOTH_CONTROLLERS
    CONTROLLER_TYPES_TO_RUN = {'paper', 'vector_field'};
else
    CONTROLLER_TYPES_TO_RUN = {CONTROLLER_TYPE};
end

for controller_run_idx = 1:length(CONTROLLER_TYPES_TO_RUN)
    CONTROLLER_TYPE = CONTROLLER_TYPES_TO_RUN{controller_run_idx};

    % ============================================================
    % Output folders
    % ============================================================
    % The two controller families are saved in separate folders, so the same
    % main can be launched with CONTROLLER_TYPE='paper' or
    % CONTROLLER_TYPE='vector_field' without overwriting figures/tables.
    BASE_OUTPUT_DIR = 'unicycle_only_outputs';
    OUTPUT_DIR = fullfile(BASE_OUTPUT_DIR, CONTROLLER_TYPE);
    FIG_DIR = fullfile(OUTPUT_DIR, 'figures');
    TABLE_DIR = fullfile(OUTPUT_DIR, 'tables');
    VIDEO_DIR = fullfile(OUTPUT_DIR, 'videos');

    ensure_dir_local(BASE_OUTPUT_DIR);
    ensure_dir_local(OUTPUT_DIR);
    ensure_dir_local(FIG_DIR);
    ensure_dir_local(TABLE_DIR);
    ensure_dir_local(VIDEO_DIR);

    fprintf('\n============================================================\n');
    fprintf('Running controller type: %s\n', CONTROLLER_TYPE);
    fprintf('Output directory: %s\n', OUTPUT_DIR);
    fprintf('============================================================\n\n');

% ============================================================
% Shared setup
% ============================================================
N = 6;

% State rows: [x_i, y_i, theta_i]
initial_unicycle_states = [ ...
    -3.0,  2.0,  0.2; ...
    -1.5, -2.0,  1.2; ...
     0.5,  3.0, -0.5; ...
     3.0, -1.0,  2.5; ...
     4.0,  1.5, -2.0; ...
    -2.0, -3.0,  0.8];

P0 = initial_unicycle_states(:, 1:2);
initial_centroid = mean(P0, 1);

% Unicycle control parameters.
% k is the gain of the Cartesian consensus vector field.
params.k = 1.0;
params.kv = 1.0;
params.ktheta = 3.0;
params.vmax = 1.5;
params.omegamax = 4.0;
params.saturate = true;
params.forward_only = true;
params.velocity_epsilon = 1e-8;
params.controller_type = CONTROLLER_TYPE;
params.paper_omega_gain = 1.0;  % gain used in the direct paper angular law

fprintf('Controller type: %s\n', params.controller_type);
fprintf('  vector_field = delayed Cartesian consensus + heading tracking\n');
fprintf('  paper        = force projection on unicycle heading, paper-inspired\n\n');

T = 35.0;
dt = 0.02;
eps_convergence = 1e-2;

A_ring = adjacency_matrix_local('ring', N);
L_ring = laplacian_matrix_local(A_ring);
info_ring = graph_spectral_info_local(L_ring, params.k);

fprintf('========== BASE GRAPH: RING ==========%s', newline);
fprintf('lambda_2   = %.8f\n', info_ring.lambda_2);
fprintf('lambda_max = %.8f\n', info_ring.lambda_max);
fprintf('tau_crit(full-state reference) = %.8f\n\n', info_ring.tau_crit);

write_cell_csv_local(fullfile(TABLE_DIR, 'graph_info_ring.csv'), ...
    {'graph','lambda_2','lambda_max','tau_crit_full_state_reference','connected'}, ...
    {'ring', info_ring.lambda_2, info_ring.lambda_max, info_ring.tau_crit, info_ring.is_connected});

if MAKE_FIGURES
    plot_graph_custom_local(A_ring, 'Ring communication graph');
    save_current_figure_local(fullfile(FIG_DIR, 'u00_ring_graph.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U1 - Baseline full-state delayed unicycle rendezvous
% ============================================================
fprintf('========== U1: BASELINE FULL-STATE DELAY UNICYCLE ==========%s', newline);

tau_u1 = 0.30 * info_ring.tau_crit;
[time_u1, states_u1, commands_u1, diag_u1] = simulate_unicycle_delay_model_local( ...
    A_ring, initial_unicycle_states, 'full_state', tau_u1, T, dt, params);
summary_u1 = summarize_unicycle_run_local(time_u1, states_u1, commands_u1, eps_convergence);
print_summary_local('U1 full-state', tau_u1, info_ring.tau_crit, summary_u1);

write_cell_csv_local(fullfile(TABLE_DIR, 'u01_baseline_full_state.csv'), ...
    {'model','tau','tau_crit','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time','final_centroid_x','final_centroid_y','path_length_mean'}, ...
    {'full_state', tau_u1, info_ring.tau_crit, tau_u1/info_ring.tau_crit, summary_u1.final_disagreement, summary_u1.max_disagreement, summary_u1.convergence_time, summary_u1.final_centroid(1), summary_u1.final_centroid(2), summary_u1.mean_path_length});

if MAKE_FIGURES
    plot_unicycle_trajectories_local(states_u1, 'U1: full-state delayed unicycle rendezvous', initial_centroid);
    save_current_figure_local(fullfile(FIG_DIR, 'u01_full_state_trajectories.png'), CLOSE_FIGURES_AFTER_SAVE);

    plot_disagreement_curves_local(time_u1, diag_u1.disagreement, {'full-state'}, 'U1: full-state disagreement');
    save_current_figure_local(fullfile(FIG_DIR, 'u01_full_state_disagreement.png'), CLOSE_FIGURES_AFTER_SAVE);
end

if MAKE_VIDEOS
    save_unicycle_animation_local(time_u1, states_u1, fullfile(VIDEO_DIR, 'u01_full_state.mp4'), 'U1 full-state');
end

% ============================================================
% U2 - Baseline neighbor-only delayed unicycle rendezvous
% ============================================================
fprintf('\n========== U2: BASELINE NEIGHBOR-ONLY DELAY UNICYCLE ==========%s', newline);

tau_u2 = tau_u1;
[time_u2, states_u2, commands_u2, diag_u2] = simulate_unicycle_delay_model_local( ...
    A_ring, initial_unicycle_states, 'neighbor_only', tau_u2, T, dt, params);
summary_u2 = summarize_unicycle_run_local(time_u2, states_u2, commands_u2, eps_convergence);
print_summary_local('U2 neighbor-only', tau_u2, info_ring.tau_crit, summary_u2);

write_cell_csv_local(fullfile(TABLE_DIR, 'u02_baseline_neighbor_only.csv'), ...
    {'model','tau','tau_crit_reference','tau_over_taucrit_reference','final_disagreement','max_disagreement','convergence_time','final_centroid_x','final_centroid_y','path_length_mean'}, ...
    {'neighbor_only', tau_u2, info_ring.tau_crit, tau_u2/info_ring.tau_crit, summary_u2.final_disagreement, summary_u2.max_disagreement, summary_u2.convergence_time, summary_u2.final_centroid(1), summary_u2.final_centroid(2), summary_u2.mean_path_length});

if MAKE_FIGURES
    plot_unicycle_trajectories_local(states_u2, 'U2: neighbor-only delayed unicycle rendezvous', initial_centroid);
    save_current_figure_local(fullfile(FIG_DIR, 'u02_neighbor_only_trajectories.png'), CLOSE_FIGURES_AFTER_SAVE);

    plot_disagreement_curves_local(time_u2, [diag_u1.disagreement(:), diag_u2.disagreement(:)], ...
        {'full-state','neighbor-only'}, 'U2: delay model comparison at same tau');
    save_current_figure_local(fullfile(FIG_DIR, 'u02_model_comparison_disagreement.png'), CLOSE_FIGURES_AFTER_SAVE);
end


% ============================================================
% U3 - Tau sweep: full-state vs neighbor-only
% ============================================================
fprintf('\n========== U3: TAU SWEEP, FULL-STATE VS NEIGHBOR-ONLY ==========%s', newline);

tau_multipliers = [0.0, 0.25, 0.50, 0.75, 0.95, 1.05, 1.20, 1.50, 2.00];
models = {'full_state', 'neighbor_only'};
tau_rows = {};
full_curve_last = [];
neigh_curve_last = [];
time_sweep_ref = [];

for m = 1:length(models)
    model = models{m};
    for idx = 1:length(tau_multipliers)
        mult = tau_multipliers(idx);
        tau_test = mult * info_ring.tau_crit;
        [time_s, states_s, commands_s, diag_s] = simulate_unicycle_delay_model_local( ...
            A_ring, initial_unicycle_states, model, tau_test, T, dt, params);
        summary_s = summarize_unicycle_run_local(time_s, states_s, commands_s, eps_convergence);
        practical_stable = practical_stability_flag_local(time_s, diag_s.disagreement);

        tau_rows(end+1, :) = {model, mult, tau_test, info_ring.tau_crit, ...
            summary_s.final_disagreement, summary_s.max_disagreement, summary_s.convergence_time, ...
            practical_stable, summary_s.final_centroid(1), summary_s.final_centroid(2), summary_s.mean_path_length};

        fprintf('%-14s | tau/taucrit = %4.2f | final disagreement = %.4e | tc = %.3f | stable_flag = %d\n', ...
            model, mult, summary_s.final_disagreement, summary_s.convergence_time, practical_stable);

        if abs(mult - 1.20) < 1e-12
            time_sweep_ref = time_s;
            if strcmp(model, 'full_state')
                full_curve_last = diag_s.disagreement(:);
                states_u3_full_12 = states_s;   % salvato per il video U3
                time_u3_12 = time_s;
            else
                neigh_curve_last = diag_s.disagreement(:);
                states_u3_neigh_12 = states_s;  % salvato per il video U3
            end
        end
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u03_tau_sweep_full_vs_neighbor.csv'), ...
    {'model','tau_multiplier','tau','tau_crit_reference','final_disagreement','max_disagreement','convergence_time','practical_stable_flag','final_centroid_x','final_centroid_y','path_length_mean'}, ...
    tau_rows);

if MAKE_FIGURES
    plot_tau_sweep_summary_local(tau_rows, 'U3: tau sweep summary');
    save_current_figure_local(fullfile(FIG_DIR, 'u03_tau_sweep_summary.png'), CLOSE_FIGURES_AFTER_SAVE);

    if ~isempty(full_curve_last) && ~isempty(neigh_curve_last)
        plot_disagreement_curves_local(time_sweep_ref, [full_curve_last, neigh_curve_last], ...
            {'full-state, 1.2 taucrit','neighbor-only, 1.2 taucrit'}, ...
            'U3: supra-critical comparison');
        save_current_figure_local(fullfile(FIG_DIR, 'u03_supracritical_curves.png'), CLOSE_FIGURES_AFTER_SAVE);
    end
end

% U3 videos: full-state instabile vs neighbor-only stabile a 1.2 taucrit
if MAKE_VIDEOS
    if exist('states_u3_full_12', 'var')
        save_unicycle_animation_local(time_u3_12, states_u3_full_12, ...
            fullfile(VIDEO_DIR, 'u03_full_state_supracritical.mp4'), ...
            'U3 full-state 1.2 taucrit (instabile)');
    end
    if exist('states_u3_neigh_12', 'var')
        save_unicycle_animation_local(time_u3_12, states_u3_neigh_12, ...
            fullfile(VIDEO_DIR, 'u03_neighbor_only_supracritical.mp4'), ...
            'U3 neighbor-only 1.2 taucrit (stabile)');
    end
end

% ============================================================
% U4 - Topology comparison adapted to unicycle dynamics
% ============================================================
fprintf('\n========== U4: TOPOLOGY COMPARISON, UNICYCLE ==========%s', newline);

graph_types = {'path', 'ring', 'star', 'complete'};
topology_rows = {};

for gidx = 1:length(graph_types)
    gname = graph_types{gidx};
    A_g = adjacency_matrix_local(gname, N);
    L_g = laplacian_matrix_local(A_g);
    info_g = graph_spectral_info_local(L_g, params.k);

    % Use the same relative delay for each topology.
    tau_g = 0.50 * info_g.tau_crit;

    for m = 1:length(models)
        model = models{m};
        [time_g, states_g, commands_g, diag_g] = simulate_unicycle_delay_model_local( ...
            A_g, initial_unicycle_states, model, tau_g, T, dt, params);
        summary_g = summarize_unicycle_run_local(time_g, states_g, commands_g, eps_convergence);

        topology_rows(end+1, :) = {gname, model, sum(A_g(:))/2, info_g.lambda_2, info_g.lambda_max, ...
            info_g.tau_crit, tau_g, tau_g/info_g.tau_crit, summary_g.final_disagreement, ...
            summary_g.max_disagreement, summary_g.convergence_time, summary_g.mean_path_length};

        fprintf('%-8s | %-14s | lambda2=%.3f | lmax=%.3f | taucrit=%.3f | tc=%.3f | final=%.3e\n', ...
            gname, model, info_g.lambda_2, info_g.lambda_max, info_g.tau_crit, summary_g.convergence_time, summary_g.final_disagreement);

        if MAKE_FIGURES && strcmp(model, 'full_state')
            plot_unicycle_trajectories_local(states_g, ['U4: topology ' gname ' - full-state'], initial_centroid);
            save_current_figure_local(fullfile(FIG_DIR, ['u04_topology_' gname '_full_state.png']), CLOSE_FIGURES_AFTER_SAVE);
        end
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u04_topology_comparison_unicycle.csv'), ...
    {'graph','model','num_edges','lambda_2','lambda_max','tau_crit','tau_used','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    topology_rows);

if MAKE_FIGURES
    plot_topology_summary_local(topology_rows, 'U4: topology comparison');
    save_current_figure_local(fullfile(FIG_DIR, 'u04_topology_summary.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U5 - Weighted graphs adapted to unicycle dynamics
% ============================================================
fprintf('\n========== U5: WEIGHTED GRAPH, UNICYCLE ==========%s', newline);

A_weighted = apply_symmetric_random_weights_local(A_ring, 0.4, 1.8, 17);
L_weighted = laplacian_matrix_local(A_weighted);
info_weighted = graph_spectral_info_local(L_weighted, params.k);
tau_weighted = 0.75 * info_weighted.tau_crit;
weighted_rows = {};

for m = 1:length(models)
    model = models{m};
    [time_w, states_w, commands_w, diag_w] = simulate_unicycle_delay_model_local( ...
        A_weighted, initial_unicycle_states, model, tau_weighted, T, dt, params);
    summary_w = summarize_unicycle_run_local(time_w, states_w, commands_w, eps_convergence);

    weighted_rows(end+1, :) = {model, sum(A_weighted(:))/2, info_weighted.lambda_2, info_weighted.lambda_max, ...
        info_weighted.tau_crit, tau_weighted, tau_weighted/info_weighted.tau_crit, ...
        summary_w.final_disagreement, summary_w.max_disagreement, summary_w.convergence_time, summary_w.mean_path_length};

    fprintf('%-14s | weighted ring | lambda2=%.3f | lmax=%.3f | taucrit=%.3f | final=%.3e\n', ...
        model, info_weighted.lambda_2, info_weighted.lambda_max, info_weighted.tau_crit, summary_w.final_disagreement);
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u05_weighted_graph_unicycle.csv'), ...
    {'model','weighted_edge_sum','lambda_2','lambda_max','tau_crit','tau_used','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    weighted_rows);

if MAKE_FIGURES
    plot_graph_custom_local(A_weighted, 'U5: weighted ring graph');
    save_current_figure_local(fullfile(FIG_DIR, 'u05_weighted_graph.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U6 - Random connected graphs adapted to unicycle dynamics
% ============================================================
fprintf('\n========== U6: RANDOM CONNECTED GRAPHS, UNICYCLE ==========%s', newline);

num_random_trials = 20;
p_edge = 0.45;
random_rows = {};

for trial = 1:num_random_trials
    A_rand = random_connected_graph_local(N, p_edge, 5000, 1000 + trial);
    L_rand = laplacian_matrix_local(A_rand);
    info_rand = graph_spectral_info_local(L_rand, params.k);
    tau_rand = 0.50 * info_rand.tau_crit;

    [time_r, states_r, commands_r, diag_r] = simulate_unicycle_delay_model_local( ...
        A_rand, initial_unicycle_states, 'full_state', tau_rand, T, dt, params);
    summary_r = summarize_unicycle_run_local(time_r, states_r, commands_r, eps_convergence);

    random_rows(end+1, :) = {trial, sum(A_rand(:))/2, info_rand.lambda_2, info_rand.lambda_max, ...
        info_rand.tau_crit, tau_rand, tau_rand/info_rand.tau_crit, summary_r.final_disagreement, ...
        summary_r.max_disagreement, summary_r.convergence_time, summary_r.mean_path_length};
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u06_random_connected_graphs_unicycle.csv'), ...
    {'trial','num_edges','lambda_2','lambda_max','tau_crit','tau_used','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    random_rows);

if MAKE_FIGURES
    plot_random_graph_summary_local(random_rows, 'U6: random connected graphs');
    save_current_figure_local(fullfile(FIG_DIR, 'u06_random_graph_summary.png'), CLOSE_FIGURES_AFTER_SAVE);
end
fprintf('Random connected graph trials completed: %d\n', num_random_trials);

% ============================================================
% U7 - Random geometric graphs adapted to unicycle dynamics
% ============================================================
fprintf('\n========== U7: RANDOM GEOMETRIC GRAPHS, UNICYCLE ==========%s', newline);

radii = [4.0, 5.0, 6.0];
trials_per_radius = 8;
geometric_rows = {};

for ridx = 1:length(radii)
    radius = radii(ridx);
    for trial = 1:trials_per_radius
        [A_geo, pos_geo] = random_geometric_graph_local(N, radius, 8.0, 5000, 2000 + 100*ridx + trial);
        L_geo = laplacian_matrix_local(A_geo);
        info_geo = graph_spectral_info_local(L_geo, params.k);
        tau_geo = 0.50 * info_geo.tau_crit;

        [time_geo, states_geo, commands_geo, diag_geo] = simulate_unicycle_delay_model_local( ...
            A_geo, initial_unicycle_states, 'full_state', tau_geo, T, dt, params);
        summary_geo = summarize_unicycle_run_local(time_geo, states_geo, commands_geo, eps_convergence);

        geometric_rows(end+1, :) = {radius, trial, sum(A_geo(:))/2, info_geo.lambda_2, info_geo.lambda_max, ...
            info_geo.tau_crit, tau_geo, tau_geo/info_geo.tau_crit, summary_geo.final_disagreement, ...
            summary_geo.max_disagreement, summary_geo.convergence_time, summary_geo.mean_path_length};
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u07_random_geometric_graphs_unicycle.csv'), ...
    {'radius','trial','num_edges','lambda_2','lambda_max','tau_crit','tau_used','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    geometric_rows);

if MAKE_FIGURES
    plot_geometric_summary_local(geometric_rows, radii, 'U7: random geometric graphs');
    save_current_figure_local(fullfile(FIG_DIR, 'u07_random_geometric_summary.png'), CLOSE_FIGURES_AFTER_SAVE);
end
fprintf('Random geometric graph trials completed: %d\n', length(radii)*trials_per_radius);

% ============================================================
% U8 - Disconnected graph: cluster rendezvous, not global rendezvous
% ============================================================
fprintf('\n========== U8: DISCONNECTED GRAPH, UNICYCLE ==========%s', newline);

A_disc = adjacency_matrix_local('disconnected', N);
L_disc = laplacian_matrix_local(A_disc);
info_disc = graph_spectral_info_local(L_disc, params.k);
tau_disc = 0.30 * info_ring.tau_crit;  % use the ring reference just to keep a finite delay

disconnected_rows = {};
for m = 1:length(models)
    model = models{m};
    [time_d, states_d, commands_d, diag_d] = simulate_unicycle_delay_model_local( ...
        A_disc, initial_unicycle_states, model, tau_disc, T, dt, params);
    summary_d = summarize_unicycle_run_local(time_d, states_d, commands_d, eps_convergence);

    disconnected_rows(end+1, :) = {model, info_disc.is_connected, info_disc.lambda_2, info_disc.lambda_max, ...
        tau_disc, summary_d.final_disagreement, summary_d.max_disagreement, summary_d.convergence_time, ...
        summary_d.final_centroid(1), summary_d.final_centroid(2)};

    fprintf('%-14s | connected=%d | lambda2=%.3f | final global disagreement=%.3e\n', ...
        model, info_disc.is_connected, info_disc.lambda_2, summary_d.final_disagreement);

    if MAKE_FIGURES && strcmp(model, 'full_state')
        plot_unicycle_trajectories_local(states_d, 'U8: disconnected graph - cluster rendezvous', []);
        save_current_figure_local(fullfile(FIG_DIR, 'u08_disconnected_full_state.png'), CLOSE_FIGURES_AFTER_SAVE);
    end

    % U8 video: un video per ogni modello
    if MAKE_VIDEOS
        save_unicycle_animation_local(time_d, states_d, ...
            fullfile(VIDEO_DIR, ['u08_disconnected_' model '.mp4']), ...
            ['U8 disconnected graph - ' model]);
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u08_disconnected_graph_unicycle.csv'), ...
    {'model','connected','lambda_2','lambda_max','tau_used','final_global_disagreement','max_disagreement','convergence_time','final_centroid_x','final_centroid_y'}, ...
    disconnected_rows);

% ============================================================
% U9 - Saturated vs unsaturated unicycle commands
% ============================================================
fprintf('\n========== U9: SATURATED VS UNSATURATED UNICYCLE ==========%s', newline);

sat_rows = {};
tau_sat = 0.60 * info_ring.tau_crit;
for sat_flag = [true, false]
    params_sat = params;
    params_sat.saturate = sat_flag;
    [time_sat, states_sat, commands_sat, diag_sat] = simulate_unicycle_delay_model_local( ...
        A_ring, initial_unicycle_states, 'full_state', tau_sat, T, dt, params_sat);
    summary_sat = summarize_unicycle_run_local(time_sat, states_sat, commands_sat, eps_convergence);
    max_v = max(max(abs(squeeze(commands_sat(:,:,1)))));
    max_omega = max(max(abs(squeeze(commands_sat(:,:,2)))));

    if sat_flag
        sat_name = 'saturated';
    else
        sat_name = 'unsaturated';
    end

    sat_rows(end+1, :) = {sat_name, tau_sat, tau_sat/info_ring.tau_crit, summary_sat.final_disagreement, ...
        summary_sat.max_disagreement, summary_sat.convergence_time, max_v, max_omega, summary_sat.mean_path_length};

    fprintf('%-12s | final disagreement=%.3e | tc=%.3f | max|v|=%.3f | max|omega|=%.3f\n', ...
        sat_name, summary_sat.final_disagreement, summary_sat.convergence_time, max_v, max_omega);

    if MAKE_FIGURES
        plot_unicycle_trajectories_local(states_sat, ['U9: ' sat_name ' commands'], initial_centroid);
        save_current_figure_local(fullfile(FIG_DIR, ['u09_' sat_name '_trajectories.png']), CLOSE_FIGURES_AFTER_SAVE);
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u09_saturated_vs_unsaturated.csv'), ...
    {'case','tau','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time','max_abs_v','max_abs_omega','path_length_mean'}, ...
    sat_rows);

% ============================================================
% U10 - Integration step sensitivity, unicycle only
% ============================================================
fprintf('\n========== U10: INTEGRATION STEP SENSITIVITY ==========%s', newline);

dt_values = [0.005, 0.01, 0.02, 0.05, 0.10];
dt_rows = {};
tau_dt = 0.50 * info_ring.tau_crit;
for didx = 1:length(dt_values)
    dt_test = dt_values(didx);
    [time_dt, states_dt, commands_dt, diag_dt] = simulate_unicycle_delay_model_local( ...
        A_ring, initial_unicycle_states, 'full_state', tau_dt, T, dt_test, params);
    summary_dt = summarize_unicycle_run_local(time_dt, states_dt, commands_dt, eps_convergence);

    dt_rows(end+1, :) = {dt_test, tau_dt, tau_dt/info_ring.tau_crit, summary_dt.final_disagreement, ...
        summary_dt.max_disagreement, summary_dt.convergence_time, summary_dt.mean_path_length};

    fprintf('dt=%6.3f | final disagreement=%.3e | tc=%.3f\n', ...
        dt_test, summary_dt.final_disagreement, summary_dt.convergence_time);
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u10_integration_step_sensitivity.csv'), ...
    {'dt','tau','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    dt_rows);

if MAKE_FIGURES
    plot_dt_summary_local(dt_rows, 'U10: integration step sensitivity');
    save_current_figure_local(fullfile(FIG_DIR, 'u10_dt_sensitivity.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U11 - Gain vs tau_crit study
% ============================================================
fprintf('\n========== U11: CONSENSUS GAIN VS TAU_CRIT ==========%s', newline);

gain_values = [0.5, 1.0, 1.5, 2.0];
gain_rows = {};
for gidx = 1:length(gain_values)
    k_test = gain_values(gidx);
    params_gain = params;
    params_gain.k = k_test;
    info_gain = graph_spectral_info_local(L_ring, k_test);
    tau_gain = 0.80 * info_gain.tau_crit;

    [time_gain, states_gain, commands_gain, diag_gain] = simulate_unicycle_delay_model_local( ...
        A_ring, initial_unicycle_states, 'full_state', tau_gain, T, dt, params_gain);
    summary_gain = summarize_unicycle_run_local(time_gain, states_gain, commands_gain, eps_convergence);

    gain_rows(end+1, :) = {k_test, info_gain.lambda_max, info_gain.tau_crit, tau_gain, ...
        tau_gain/info_gain.tau_crit, summary_gain.final_disagreement, summary_gain.max_disagreement, ...
        summary_gain.convergence_time, summary_gain.mean_path_length};

    fprintf('k=%.2f | taucrit=%.4f | tau=%.4f | final=%.3e | tc=%.3f\n', ...
        k_test, info_gain.tau_crit, tau_gain, summary_gain.final_disagreement, summary_gain.convergence_time);
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u11_gain_vs_taucrit.csv'), ...
    {'consensus_gain_k','lambda_max','tau_crit','tau_used','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    gain_rows);

if MAKE_FIGURES
    plot_gain_summary_local(gain_rows, 'U11: gain vs taucrit');
    save_current_figure_local(fullfile(FIG_DIR, 'u11_gain_vs_taucrit.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U12 - Initial heading sensitivity
% ============================================================
fprintf('\n========== U12: INITIAL HEADING SENSITIVITY ==========%s', newline);

heading_cases = {'original', 'all_zero', 'opposite_to_motion', 'random'};
heading_rows = {};
tau_heading = 0.50 * info_ring.tau_crit;

for hidx = 1:length(heading_cases)
    hcase = heading_cases{hidx};
    states0_h = initial_unicycle_states;
    if strcmp(hcase, 'all_zero')
        states0_h(:,3) = 0;
    elseif strcmp(hcase, 'opposite_to_motion')
        for i = 1:N
            desired_to_centroid = initial_centroid - states0_h(i,1:2);
            states0_h(i,3) = wrap_to_pi_local(atan2(desired_to_centroid(2), desired_to_centroid(1)) + pi);
        end
    elseif strcmp(hcase, 'random')
        safe_rng_local(77);
        states0_h(:,3) = -pi + 2*pi*rand(N,1);
        safe_rng_local(7);
    end

    [time_h, states_h, commands_h, diag_h] = simulate_unicycle_delay_model_local( ...
        A_ring, states0_h, 'full_state', tau_heading, T, dt, params);
    summary_h = summarize_unicycle_run_local(time_h, states_h, commands_h, eps_convergence);

    heading_rows(end+1, :) = {hcase, tau_heading, tau_heading/info_ring.tau_crit, summary_h.final_disagreement, ...
        summary_h.max_disagreement, summary_h.convergence_time, summary_h.mean_path_length};

    fprintf('%-18s | final disagreement=%.3e | tc=%.3f | path=%.3f\n', ...
        hcase, summary_h.final_disagreement, summary_h.convergence_time, summary_h.mean_path_length);
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u12_initial_heading_sensitivity.csv'), ...
    {'heading_case','tau','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    heading_rows);

if MAKE_FIGURES
    plot_heading_summary_local(heading_rows, 'U12: initial heading sensitivity');
    save_current_figure_local(fullfile(FIG_DIR, 'u12_heading_sensitivity.png'), CLOSE_FIGURES_AFTER_SAVE);
end


% ============================================================
% U13 - Complete graph configurations
% ============================================================
fprintf('\n========== U13: COMPLETE GRAPH CONFIGURATIONS ==========%s', newline);

complete_sizes = [4, 6, 8, 10];
complete_weights = [0.5, 1.0, 1.5];
complete_tau_multipliers = [0.50, 0.95, 1.10, 1.50];
complete_rows = {};
complete_T = 28.0;

for nidx = 1:length(complete_sizes)
    Nc = complete_sizes(nidx);
    states0_c = generate_initial_unicycle_states_local(Nc, 3000 + Nc);

    for widx = 1:length(complete_weights)
        w_complete = complete_weights(widx);
        A_complete = w_complete * (ones(Nc) - eye(Nc));
        L_complete = laplacian_matrix_local(A_complete);
        info_complete = graph_spectral_info_local(L_complete, params.k);

        for midx = 1:length(models)
            model = models{midx};
            for tidx = 1:length(complete_tau_multipliers)
                mult = complete_tau_multipliers(tidx);
                tau_complete = mult * info_complete.tau_crit;

                [time_c, states_c, commands_c, diag_c] = simulate_unicycle_delay_model_local( ...
                    A_complete, states0_c, model, tau_complete, complete_T, dt, params);
                summary_c = summarize_unicycle_run_local(time_c, states_c, commands_c, eps_convergence);
                stable_c = practical_stability_flag_local(time_c, diag_c.disagreement);

                complete_rows(end+1, :) = {Nc, w_complete, model, sum(A_complete(:))/2, ...
                    info_complete.lambda_2, info_complete.lambda_max, info_complete.tau_crit, ...
                    mult, tau_complete, summary_c.final_disagreement, summary_c.max_disagreement, ...
                    summary_c.convergence_time, stable_c, summary_c.mean_path_length};

                fprintf('K_N=%2d | w=%.1f | %-14s | tau/taucrit=%4.2f | taucrit=%.4f | final=%.3e | stable=%d\n', ...
                    Nc, w_complete, model, mult, info_complete.tau_crit, summary_c.final_disagreement, stable_c);

                if MAKE_FIGURES && Nc == 6 && abs(w_complete - 1.0) < 1e-12 && ...
                        strcmp(model, 'full_state') && abs(mult - 0.50) < 1e-12
                    plot_unicycle_trajectories_local(states_c, 'U13: complete graph K6, full-state, 0.5 taucrit', mean(states0_c(:,1:2), 1));
                    save_current_figure_local(fullfile(FIG_DIR, 'u13_complete_K6_full_state_trajectories.png'), CLOSE_FIGURES_AFTER_SAVE);
                end
            end
        end
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u13_complete_graph_configurations.csv'), ...
    {'N','complete_edge_weight','model','weighted_edge_sum','lambda_2','lambda_max','tau_crit','tau_multiplier','tau_used','final_disagreement','max_disagreement','convergence_time','practical_stable_flag','path_length_mean'}, ...
    complete_rows);

if MAKE_FIGURES
    plot_complete_graph_summary_local(complete_rows, 'U13: complete graph configurations');
    save_current_figure_local(fullfile(FIG_DIR, 'u13_complete_graph_summary.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U14 - Extension: arbitrary target rendezvous
% ============================================================
fprintf('\n========== U14: EXTENSION - ARBITRARY TARGET RENDEZVOUS ==========%s', newline);

target_point = [2.5, -1.0];
k_target = 0.65;
tau_ext = 0.40 * info_ring.tau_crit;
[time_tar, states_tar, commands_tar, diag_tar] = simulate_unicycle_target_rendezvous_local( ...
    A_ring, initial_unicycle_states, 'full_state', tau_ext, T, dt, params, target_point, k_target);
summary_tar = summarize_unicycle_run_local(time_tar, states_tar, commands_tar, eps_convergence);
target_error_tar = mean_distance_to_point_local(squeeze(states_tar(end,:,1:2)), target_point);

fprintf('target=[%.2f, %.2f] | mean final target error=%.4f | final disagreement=%.3e\n', ...
    target_point(1), target_point(2), target_error_tar, summary_tar.final_disagreement);

write_cell_csv_local(fullfile(TABLE_DIR, 'u14_extension_arbitrary_target.csv'), ...
    {'extension','model','tau','tau_over_taucrit','target_x','target_y','k_target','final_disagreement','mean_final_target_error','convergence_time','path_length_mean'}, ...
    {'arbitrary_target', 'full_state', tau_ext, tau_ext/info_ring.tau_crit, target_point(1), target_point(2), k_target, summary_tar.final_disagreement, target_error_tar, summary_tar.convergence_time, summary_tar.mean_path_length});

if MAKE_FIGURES
    plot_unicycle_trajectories_local(states_tar, 'U14: arbitrary target rendezvous', target_point);
    save_current_figure_local(fullfile(FIG_DIR, 'u14_arbitrary_target_trajectories.png'), CLOSE_FIGURES_AFTER_SAVE);
    plot_disagreement_curves_local(time_tar, diag_tar.target_error, {'mean distance to target'}, 'U14: target error');
    save_current_figure_local(fullfile(FIG_DIR, 'u14_arbitrary_target_error.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U15 - Extension: leader-follower with moving target
% ============================================================
fprintf('\n========== U15: EXTENSION - LEADER-FOLLOWER MOVING TARGET ==========%s', newline);

leader_index = 1;
k_leader = 1.4;
leader_fun = @(t) [1.8 * cos(0.18 * t), 1.8 * sin(0.18 * t)];
[time_lf, states_lf, commands_lf, diag_lf] = simulate_unicycle_leader_follower_local( ...
    A_ring, initial_unicycle_states, 'neighbor_only', tau_ext, 45.0, dt, params, leader_index, leader_fun, k_leader);
summary_lf = summarize_unicycle_run_local(time_lf, states_lf, commands_lf, eps_convergence);
lf_tail_start = max(1, round(0.80 * length(time_lf)));
leader_tracking_tail = mean(diag_lf.leader_tracking_error(lf_tail_start:end));
follower_spread_tail = mean(diag_lf.follower_spread(lf_tail_start:end));

fprintf('leader=%d | tail leader tracking error=%.4f | tail follower spread=%.4f\n', ...
    leader_index, leader_tracking_tail, follower_spread_tail);

write_cell_csv_local(fullfile(TABLE_DIR, 'u15_extension_leader_follower.csv'), ...
    {'extension','model','tau','tau_over_taucrit','leader_index','k_leader','tail_leader_tracking_error','tail_follower_spread','final_disagreement','path_length_mean'}, ...
    {'leader_follower_moving_target', 'neighbor_only', tau_ext, tau_ext/info_ring.tau_crit, leader_index, k_leader, leader_tracking_tail, follower_spread_tail, summary_lf.final_disagreement, summary_lf.mean_path_length});

if MAKE_FIGURES
    plot_unicycle_trajectories_local(states_lf, 'U15: leader-follower moving target', []);
    hold on;
    plot(diag_lf.leader_target(:,1), diag_lf.leader_target(:,2), 'k--', 'LineWidth', 1.2);
    save_current_figure_local(fullfile(FIG_DIR, 'u15_leader_follower_trajectories.png'), CLOSE_FIGURES_AFTER_SAVE);
    plot_disagreement_curves_local(time_lf, [diag_lf.leader_tracking_error(:), diag_lf.follower_spread(:)], ...
        {'leader tracking error','follower spread'}, 'U15: leader-follower errors');
    save_current_figure_local(fullfile(FIG_DIR, 'u15_leader_follower_errors.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% U15 video: rendezvous dinamico con target mobile
if MAKE_VIDEOS
    save_unicycle_animation_local(time_lf, states_lf, ...
        fullfile(VIDEO_DIR, 'u15_leader_follower.mp4'), ...
        'U15 leader-follower target mobile');
end

% ============================================================
% U16 - Extension: rigid formation with unicycle agents
% ============================================================
fprintf('\n========== U16: EXTENSION - RIGID FORMATION ==========%s', newline);

formation_offsets = polygon_offsets_local(N, 1.1);
formation_center = [1.0, 1.0];
k_anchor = 0.45;
[time_form, states_form, commands_form, diag_form] = simulate_unicycle_formation_local( ...
    A_ring, initial_unicycle_states, 'full_state', tau_ext, 40.0, dt, params, formation_offsets, formation_center, k_anchor);
summary_form = summarize_unicycle_run_local(time_form, states_form, commands_form, eps_convergence);
formation_error_final = diag_form.formation_error(end);

fprintf('formation center=[%.2f, %.2f] | final formation error=%.4f | final centroid=[%.3f, %.3f]\n', ...
    formation_center(1), formation_center(2), formation_error_final, summary_form.final_centroid(1), summary_form.final_centroid(2));

write_cell_csv_local(fullfile(TABLE_DIR, 'u16_extension_formation.csv'), ...
    {'extension','model','tau','tau_over_taucrit','formation_center_x','formation_center_y','k_anchor','final_formation_error','final_centroid_x','final_centroid_y','path_length_mean'}, ...
    {'rigid_formation', 'full_state', tau_ext, tau_ext/info_ring.tau_crit, formation_center(1), formation_center(2), k_anchor, formation_error_final, summary_form.final_centroid(1), summary_form.final_centroid(2), summary_form.mean_path_length});

if MAKE_FIGURES
    plot_unicycle_trajectories_local(states_form, 'U16: rigid formation', formation_center);
    save_current_figure_local(fullfile(FIG_DIR, 'u16_formation_trajectories.png'), CLOSE_FIGURES_AFTER_SAVE);
    plot_disagreement_curves_local(time_form, diag_form.formation_error, {'formation error'}, 'U16: formation error');
    save_current_figure_local(fullfile(FIG_DIR, 'u16_formation_error.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% U16 video: formazione rigida
if MAKE_VIDEOS
    save_unicycle_animation_local(time_form, states_form, ...
        fullfile(VIDEO_DIR, 'u16_rigid_formation.mp4'), ...
        'U16 rigid formation');
end

% ============================================================
% U17 - Extension: noise and disturbance robustness
% ============================================================
fprintf('\n========== U17: EXTENSION - NOISE ROBUSTNESS ==========%s', newline);

noise_levels = [0.00, 0.02, 0.05, 0.10];
noise_rows = {};
for nidx = 1:length(noise_levels)
    noise_std = noise_levels(nidx);
    [time_noise, states_noise, commands_noise, diag_noise] = simulate_unicycle_noise_local( ...
        A_ring, initial_unicycle_states, 'neighbor_only', tau_ext, T, dt, params, noise_std, 4000 + nidx);
    summary_noise = summarize_unicycle_run_local(time_noise, states_noise, commands_noise, eps_convergence);
    tail_start_noise = max(1, round(0.80 * length(time_noise)));
    residual_noise = mean(diag_noise.disagreement(tail_start_noise:end));

    noise_rows(end+1, :) = {noise_std, tau_ext, tau_ext/info_ring.tau_crit, residual_noise, ...
        summary_noise.final_disagreement, summary_noise.max_disagreement, summary_noise.convergence_time, summary_noise.mean_path_length};

    fprintf('noise_std=%.3f | residual disagreement=%.4e | final=%.4e\n', ...
        noise_std, residual_noise, summary_noise.final_disagreement);
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u17_extension_noise_robustness.csv'), ...
    {'noise_std','tau','tau_over_taucrit','tail_residual_disagreement','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    noise_rows);

if MAKE_FIGURES
    plot_noise_summary_local(noise_rows, 'U17: noise robustness');
    save_current_figure_local(fullfile(FIG_DIR, 'u17_noise_robustness.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U18 - Extension: packet loss on communication links
% ============================================================
fprintf('\n========== U18: EXTENSION - PACKET LOSS ==========%s', newline);

loss_probabilities = [0.00, 0.10, 0.25, 0.40];
loss_rows = {};
for lidx = 1:length(loss_probabilities)
    loss_prob = loss_probabilities(lidx);
    [time_loss, states_loss, commands_loss, diag_loss] = simulate_unicycle_packet_loss_local( ...
        A_ring, initial_unicycle_states, 'neighbor_only', tau_ext, T, dt, params, loss_prob, 5000 + lidx);
    summary_loss = summarize_unicycle_run_local(time_loss, states_loss, commands_loss, eps_convergence);
    tail_start_loss = max(1, round(0.80 * length(time_loss)));
    residual_loss = mean(diag_loss.disagreement(tail_start_loss:end));

    loss_rows(end+1, :) = {loss_prob, tau_ext, tau_ext/info_ring.tau_crit, residual_loss, ...
        summary_loss.final_disagreement, summary_loss.max_disagreement, summary_loss.convergence_time, summary_loss.mean_path_length};

    fprintf('loss_prob=%.2f | residual disagreement=%.4e | final=%.4e\n', ...
        loss_prob, residual_loss, summary_loss.final_disagreement);
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u18_extension_packet_loss.csv'), ...
    {'packet_loss_probability','tau','tau_over_taucrit','tail_residual_disagreement','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    loss_rows);

if MAKE_FIGURES
    plot_packet_loss_summary_local(loss_rows, 'U18: packet loss');
    save_current_figure_local(fullfile(FIG_DIR, 'u18_packet_loss.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U19 - Extension: obstacle avoidance and inter-robot safety
% ============================================================
fprintf('\n========== U19: EXTENSION - OBSTACLE AND COLLISION AVOIDANCE ==========%s', newline);

obstacle.center = [0.0, 0.0];
obstacle.radius = 0.75;
obstacle.influence_radius = 2.20;
obstacle.k_obstacle = 1.0;
safety.distance = 0.70;
safety.k_collision = 0.45;
[time_av, states_av, commands_av, diag_av] = simulate_unicycle_avoidance_local( ...
    A_ring, initial_unicycle_states, 'neighbor_only', tau_ext, 45.0, dt, params, obstacle, safety);
summary_av = summarize_unicycle_run_local(time_av, states_av, commands_av, eps_convergence);
min_pair_dist = min(diag_av.min_pairwise_distance);
min_obstacle_clearance = min(diag_av.obstacle_clearance);

fprintf('min pairwise distance=%.4f | min obstacle clearance=%.4f | final disagreement=%.3e\n', ...
    min_pair_dist, min_obstacle_clearance, summary_av.final_disagreement);

write_cell_csv_local(fullfile(TABLE_DIR, 'u19_extension_obstacle_collision_avoidance.csv'), ...
    {'extension','model','tau','tau_over_taucrit','obstacle_x','obstacle_y','obstacle_radius','safety_distance','min_pairwise_distance','min_obstacle_clearance','final_disagreement','path_length_mean'}, ...
    {'obstacle_collision_avoidance', 'neighbor_only', tau_ext, tau_ext/info_ring.tau_crit, obstacle.center(1), obstacle.center(2), obstacle.radius, safety.distance, min_pair_dist, min_obstacle_clearance, summary_av.final_disagreement, summary_av.mean_path_length});

if MAKE_FIGURES
    plot_unicycle_trajectories_local(states_av, 'U19: obstacle and collision avoidance', []);
    hold on;
    draw_circle_local(obstacle.center, obstacle.radius, 'k-');
    draw_circle_local(obstacle.center, obstacle.influence_radius, 'k--');
    save_current_figure_local(fullfile(FIG_DIR, 'u19_avoidance_trajectories.png'), CLOSE_FIGURES_AFTER_SAVE);
    plot_disagreement_curves_local(time_av, [diag_av.min_pairwise_distance(:), diag_av.obstacle_clearance(:)], ...
        {'min pairwise distance','min obstacle clearance'}, 'U19: safety metrics');
    save_current_figure_local(fullfile(FIG_DIR, 'u19_avoidance_safety_metrics.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% U19 video: obstacle e collision avoidance
if MAKE_VIDEOS
    save_unicycle_animation_local(time_av, states_av, ...
        fullfile(VIDEO_DIR, 'u19_obstacle_collision_avoidance.mp4'), ...
        'U19 obstacle and collision avoidance');
end

% ============================================================
% U20 - Extension: switching geometric graph and connectivity monitoring
% ============================================================
fprintf('\n========== U20: EXTENSION - SWITCHING GRAPH AND CONNECTIVITY ==========%s', newline);

comm_radius = 4.8;
[time_sw, states_sw, commands_sw, diag_sw] = simulate_unicycle_switching_graph_local( ...
    initial_unicycle_states, 'neighbor_only', tau_ext, 45.0, dt, params, comm_radius);
summary_sw = summarize_unicycle_run_local(time_sw, states_sw, commands_sw, eps_convergence);
min_lambda2_sw = min(diag_sw.lambda2);
mean_edges_sw = mean(diag_sw.num_edges);

fprintf('comm_radius=%.2f | min lambda2=%.4f | mean edges=%.2f | final disagreement=%.3e\n', ...
    comm_radius, min_lambda2_sw, mean_edges_sw, summary_sw.final_disagreement);

write_cell_csv_local(fullfile(TABLE_DIR, 'u20_extension_switching_connectivity.csv'), ...
    {'extension','model','tau','tau_over_taucrit_reference','communication_radius','min_lambda2','mean_num_edges','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    {'switching_geometric_graph', 'neighbor_only', tau_ext, tau_ext/info_ring.tau_crit, comm_radius, min_lambda2_sw, mean_edges_sw, summary_sw.final_disagreement, summary_sw.max_disagreement, summary_sw.convergence_time, summary_sw.mean_path_length});

if MAKE_FIGURES
    plot_unicycle_trajectories_local(states_sw, 'U20: switching geometric graph', initial_centroid);
    save_current_figure_local(fullfile(FIG_DIR, 'u20_switching_graph_trajectories.png'), CLOSE_FIGURES_AFTER_SAVE);
    plot_scalar_time_series_local(time_sw, diag_sw.lambda2, 'lambda_2(L(t))', 'U20: algebraic connectivity over time');
    save_current_figure_local(fullfile(FIG_DIR, 'u20_switching_graph_lambda2.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% U20 video: rendezvous dinamico con grafo variabile nel tempo
if MAKE_VIDEOS
    save_unicycle_animation_local(time_sw, states_sw, ...
        fullfile(VIDEO_DIR, 'u20_switching_graph.mp4'), ...
        'U20 switching geometric graph');
end


% ============================================================
% U21 - Edge-dependent communication delays
% ============================================================
fprintf('\n========== U21: EDGE-DEPENDENT DELAYS ==========%s', newline);

edge_cases = { ...
    'mild_edge_delays',          0.10, 0.50, 6101; ...
    'heterogeneous_edge_delays', 0.20, 0.95, 6102; ...
    'stress_edge_delays',        0.80, 1.30, 6103 ...
};
edge_summary_rows = {};
edge_value_rows = {};
edge_stress_time = [];
edge_stress_curves = [];

for eidx = 1:size(edge_cases, 1)
    case_name = edge_cases{eidx, 1};
    low_ratio = edge_cases{eidx, 2};
    high_ratio = edge_cases{eidx, 3};
    seed = edge_cases{eidx, 4};
    Tau_edges = generate_edge_delay_matrix_local(A_ring, info_ring.tau_crit, low_ratio, high_ratio, seed, true);
    edge_value_rows = [edge_value_rows; edge_delay_rows_local(Tau_edges, A_ring, case_name, info_ring.tau_crit)];

    for m = 1:length(models)
        model = models{m};
        [time_ed, states_ed, commands_ed, diag_ed] = simulate_unicycle_edge_dependent_delays_local( ...
            A_ring, initial_unicycle_states, model, Tau_edges, T, dt, params);
        summary_ed = summarize_unicycle_run_local(time_ed, states_ed, commands_ed, eps_convergence);
        stable_ed = practical_stability_flag_local(time_ed, diag_ed.disagreement);
        active_taus = Tau_edges(A_ring > 0);

        edge_summary_rows(end+1, :) = {case_name, model, low_ratio, high_ratio, min(active_taus), ...
            mean(active_taus), max(active_taus), min(active_taus)/info_ring.tau_crit, ...
            mean(active_taus)/info_ring.tau_crit, max(active_taus)/info_ring.tau_crit, ...
            summary_ed.final_disagreement, summary_ed.max_disagreement, summary_ed.convergence_time, ...
            stable_ed, summary_ed.mean_path_length};

        fprintf('%-26s | %-14s | tau_max/taucrit=%.2f | final=%.3e | stable=%d\n', ...
            case_name, model, max(active_taus)/info_ring.tau_crit, summary_ed.final_disagreement, stable_ed);

        if strcmp(case_name, 'stress_edge_delays')
            edge_stress_time = time_ed;
            edge_stress_curves = [edge_stress_curves, diag_ed.disagreement(:)];
        end
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u21_edge_dependent_delay_values.csv'), ...
    {'case','i','j','tau_ij','tau_ij_over_taucrit'}, edge_value_rows);
write_cell_csv_local(fullfile(TABLE_DIR, 'u21_edge_dependent_delay_summary.csv'), ...
    {'case','model','low_ratio','high_ratio','tau_min','tau_mean','tau_max','tau_min_over_taucrit','tau_mean_over_taucrit','tau_max_over_taucrit','final_disagreement','max_disagreement','convergence_time','practical_stable_flag','path_length_mean'}, ...
    edge_summary_rows);

if MAKE_FIGURES
    plot_edge_delay_summary_local(edge_summary_rows, 'U21: edge-dependent delays');
    save_current_figure_local(fullfile(FIG_DIR, 'u21_edge_dependent_delay_summary.png'), CLOSE_FIGURES_AFTER_SAVE);
    if size(edge_stress_curves, 2) == 2
        plot_disagreement_curves_local(edge_stress_time, edge_stress_curves, {'full-state','neighbor-only'}, ...
            'U21: stress edge-dependent delays');
        save_current_figure_local(fullfile(FIG_DIR, 'u21_edge_dependent_stress_curves.png'), CLOSE_FIGURES_AFTER_SAVE);
    end
end

% ============================================================
% U22 - Time-varying communication delays
% ============================================================
fprintf('\n========== U22: TIME-VARYING DELAYS ==========%s', newline);

tv_cases = { ...
    'subcritical',       0.35, 0.25, 0.04; ...
    'near_critical',     0.70, 0.25, 0.04; ...
    'crossing_critical', 0.95, 0.25, 0.04 ...
};
tv_summary_rows = {};
tv_profile_rows = {};
tv_cross_time = [];
tv_cross_curves = [];
tv_cross_tau = [];

for cidx = 1:size(tv_cases, 1)
    case_name = tv_cases{cidx, 1};
    base_ratio = tv_cases{cidx, 2};
    amplitude_ratio = tv_cases{cidx, 3};
    frequency_hz = tv_cases{cidx, 4};
    tau_fun = @(t) info_ring.tau_crit * base_ratio * (1.0 + amplitude_ratio * sin(2*pi*frequency_hz*t));

    for m = 1:length(models)
        model = models{m};
        [time_tv, states_tv, commands_tv, diag_tv] = simulate_unicycle_time_varying_delay_local( ...
            A_ring, initial_unicycle_states, model, tau_fun, T, dt, params);
        summary_tv = summarize_unicycle_run_local(time_tv, states_tv, commands_tv, eps_convergence);
        stable_tv = practical_stability_flag_local(time_tv, diag_tv.disagreement);

        tv_summary_rows(end+1, :) = {case_name, model, base_ratio, amplitude_ratio, frequency_hz, ...
            min(diag_tv.tau), mean(diag_tv.tau), max(diag_tv.tau), ...
            min(diag_tv.tau)/info_ring.tau_crit, mean(diag_tv.tau)/info_ring.tau_crit, max(diag_tv.tau)/info_ring.tau_crit, ...
            summary_tv.final_disagreement, summary_tv.max_disagreement, summary_tv.convergence_time, ...
            stable_tv, summary_tv.mean_path_length};

        fprintf('%-18s | %-14s | tau_max/taucrit=%.2f | final=%.3e | stable=%d\n', ...
            case_name, model, max(diag_tv.tau)/info_ring.tau_crit, summary_tv.final_disagreement, stable_tv);

        if strcmp(case_name, 'crossing_critical')
            tv_cross_time = time_tv;
            tv_cross_curves = [tv_cross_curves, diag_tv.disagreement(:)];
            tv_cross_tau = diag_tv.tau(:);
        end
    end

    % Save a compact profile sampled every 50 steps for readability.
    sample_idx = 1:50:length(tv_cross_time);
    if strcmp(case_name, 'crossing_critical') && ~isempty(tv_cross_time)
        for si = sample_idx
            tv_profile_rows(end+1, :) = {case_name, tv_cross_time(si), tv_cross_tau(si), tv_cross_tau(si)/info_ring.tau_crit};
        end
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u22_time_varying_delay_summary.csv'), ...
    {'case','model','base_ratio','amplitude_ratio','frequency_hz','tau_min','tau_mean','tau_max','tau_min_over_taucrit','tau_mean_over_taucrit','tau_max_over_taucrit','final_disagreement','max_disagreement','convergence_time','practical_stable_flag','path_length_mean'}, ...
    tv_summary_rows);
write_cell_csv_local(fullfile(TABLE_DIR, 'u22_time_varying_delay_profile_crossing.csv'), ...
    {'case','time','tau','tau_over_taucrit'}, tv_profile_rows);

if MAKE_FIGURES
    plot_time_varying_summary_local(tv_summary_rows, 'U22: time-varying delays');
    save_current_figure_local(fullfile(FIG_DIR, 'u22_time_varying_delay_summary.png'), CLOSE_FIGURES_AFTER_SAVE);
    if size(tv_cross_curves, 2) == 2
        plot_disagreement_curves_local(tv_cross_time, tv_cross_curves, {'full-state','neighbor-only'}, ...
            'U22: crossing-critical time-varying delay');
        save_current_figure_local(fullfile(FIG_DIR, 'u22_crossing_critical_curves.png'), CLOSE_FIGURES_AFTER_SAVE);
        plot_scalar_time_series_local(tv_cross_time, tv_cross_tau/info_ring.tau_crit, 'tau(t) / taucrit', ...
            'U22: crossing-critical delay profile');
        save_current_figure_local(fullfile(FIG_DIR, 'u22_crossing_critical_tau_profile.png'), CLOSE_FIGURES_AFTER_SAVE);
    end
end

% ============================================================
% U23 - Potential-field edge-clearance formation
% ============================================================
fprintf('\n========== U23: POTENTIAL-FIELD EDGE-CLEARANCE FORMATION ==========%s', newline);

potential_rows = {};
desired_clearance = 1.35;
potential_error_curves = [];
potential_time = [];
for m = 1:length(models)
    model = models{m};
    [time_pf, states_pf, commands_pf, diag_pf] = simulate_unicycle_potential_clearance_local( ...
        A_ring, initial_unicycle_states, model, tau_ext, 45.0, dt, params, desired_clearance, ...
        0.65, initial_centroid, 0.05);
    summary_pf = summarize_unicycle_run_local(time_pf, states_pf, commands_pf, eps_convergence);
    tail_start_pf = max(1, round(0.80 * length(time_pf)));
    tail_edge_error = mean(diag_pf.edge_distance_error(tail_start_pf:end));

    potential_rows(end+1, :) = {'potential_field_edge_clearance', model, tau_ext, tau_ext/info_ring.tau_crit, ...
        desired_clearance, diag_pf.edge_distance_error(end), tail_edge_error, ...
        summary_pf.final_disagreement, summary_pf.max_disagreement, summary_pf.convergence_time, summary_pf.mean_path_length};

    potential_time = time_pf;
    potential_error_curves = [potential_error_curves, diag_pf.edge_distance_error(:)];

    fprintf('%-14s | desired_clearance=%.2f | tail edge error=%.4e | final disagreement=%.3e\n', ...
        model, desired_clearance, tail_edge_error, summary_pf.final_disagreement);

    if MAKE_FIGURES
        plot_unicycle_trajectories_local(states_pf, ['U23: potential-field clearance formation - ' model], initial_centroid);
        save_current_figure_local(fullfile(FIG_DIR, ['u23_potential_clearance_' model '_trajectories.png']), CLOSE_FIGURES_AFTER_SAVE);
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u23_potential_field_edge_clearance.csv'), ...
    {'extension','model','tau','tau_over_taucrit_reference','desired_clearance','final_edge_distance_error','tail_edge_distance_error','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    potential_rows);

if MAKE_FIGURES
    plot_disagreement_curves_local(potential_time, potential_error_curves, {'full-state','neighbor-only'}, ...
        'U23: potential-field edge-distance error');
    save_current_figure_local(fullfile(FIG_DIR, 'u23_potential_clearance_error.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U24 - Switching obstacle avoidance
% ============================================================
fprintf('\n========== U24: SWITCHING OBSTACLE AVOIDANCE ==========%s', newline);

switch_obstacle.center = [0.0, 0.0];
switch_obstacle.radius = 0.75;
switch_obstacle.influence_radius = 2.20;
switch_rows = {};
switch_clearance_curves = [];
switch_time = [];

for m = 1:length(models)
    model = models{m};
    [time_so, states_so, commands_so, diag_so] = simulate_unicycle_switching_obstacle_controller_local( ...
        A_ring, initial_unicycle_states, model, tau_ext, 45.0, dt, params, switch_obstacle, 2.20, 1.05);
    summary_so = summarize_unicycle_run_local(time_so, states_so, commands_so, eps_convergence);
    min_clearance_so = min(diag_so.obstacle_clearance);
    total_activations = sum(diag_so.avoidance_active_count);

    switch_rows(end+1, :) = {'switching_obstacle_avoidance', model, tau_ext, tau_ext/info_ring.tau_crit, ...
        min_clearance_so, total_activations, summary_so.final_disagreement, summary_so.max_disagreement, ...
        summary_so.convergence_time, summary_so.mean_path_length};

    switch_time = time_so;
    switch_clearance_curves = [switch_clearance_curves, diag_so.obstacle_clearance(:)];

    fprintf('%-14s | min obstacle clearance=%.4f | activations=%d | final disagreement=%.3e\n', ...
        model, min_clearance_so, total_activations, summary_so.final_disagreement);

    if MAKE_FIGURES
        plot_unicycle_trajectories_local(states_so, ['U24: switching obstacle avoidance - ' model], []);
        hold on;
        draw_circle_local(switch_obstacle.center, switch_obstacle.radius, 'k-');
        draw_circle_local(switch_obstacle.center, switch_obstacle.influence_radius, 'k--');
        save_current_figure_local(fullfile(FIG_DIR, ['u24_switching_obstacle_' model '_trajectories.png']), CLOSE_FIGURES_AFTER_SAVE);
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u24_switching_obstacle_avoidance.csv'), ...
    {'extension','model','tau','tau_over_taucrit_reference','min_obstacle_clearance','total_avoidance_activations','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    switch_rows);

if MAKE_FIGURES
    plot_disagreement_curves_local(switch_time, switch_clearance_curves, {'full-state','neighbor-only'}, ...
        'U24: switching obstacle clearance');
    save_current_figure_local(fullfile(FIG_DIR, 'u24_switching_obstacle_clearance.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U25 - Initial-history protocol for delay systems
% ============================================================
fprintf('\n========== U25: INITIAL-HISTORY PROTOCOL ==========%s', newline);

history_modes = {'constant', 'zero_prehistory'};
tau_hist = 0.80 * info_ring.tau_crit;
history_rows = {};
for m = 1:length(models)
    model = models{m};
    for hidx = 1:length(history_modes)
        history_mode = history_modes{hidx};
        [time_hi, states_hi, commands_hi, diag_hi] = simulate_unicycle_delay_model_history_local( ...
            A_ring, initial_unicycle_states, model, tau_hist, T, dt, params, history_mode);
        summary_hi = summarize_unicycle_run_local(time_hi, states_hi, commands_hi, eps_convergence);

        history_rows(end+1, :) = {'initial_history_protocol', model, history_mode, tau_hist, tau_hist/info_ring.tau_crit, ...
            summary_hi.final_centroid(1), summary_hi.final_centroid(2), summary_hi.final_disagreement, ...
            summary_hi.max_disagreement, summary_hi.convergence_time, summary_hi.mean_path_length};

        fprintf('%-14s | history=%-16s | final disagreement=%.3e | centroid=[%.3f %.3f]\n', ...
            model, history_mode, summary_hi.final_disagreement, summary_hi.final_centroid(1), summary_hi.final_centroid(2));

        if MAKE_FIGURES
            plot_unicycle_trajectories_local(states_hi, ['U25: history=' history_mode ', model=' model], initial_centroid);
            save_current_figure_local(fullfile(FIG_DIR, ['u25_history_' model '_' history_mode '_trajectories.png']), CLOSE_FIGURES_AFTER_SAVE);
        end
    end
end

write_cell_csv_local(fullfile(TABLE_DIR, 'u25_initial_history_protocol.csv'), ...
    {'extension','model','history_mode','tau','tau_over_taucrit_reference','final_centroid_x','final_centroid_y','final_disagreement','max_disagreement','convergence_time','path_length_mean'}, ...
    history_rows);

if MAKE_FIGURES
    plot_history_summary_local(history_rows, 'U25: initial delay pre-history');
    save_current_figure_local(fullfile(FIG_DIR, 'u25_initial_history_summary.png'), CLOSE_FIGURES_AFTER_SAVE);
end

% ============================================================
% U26 - Extension summary table
% ============================================================

extension_summary_rows = { ...
    'U14', 'arbitrary_target', 'forces rendezvous to a prescribed point instead of the initial centroid'; ...
    'U15', 'leader_follower_moving_target', 'one unicycle tracks a moving reference while the others follow through delayed consensus'; ...
    'U16', 'rigid_formation', 'consensus is applied to shifted coordinates to obtain a spatial formation'; ...
    'U17', 'noise_robustness', 'adds measurement, command and process noise to study residual disagreement'; ...
    'U18', 'packet_loss', 'randomly removes communication links at each step'; ...
    'U19', 'obstacle_collision_avoidance', 'adds continuous repulsive terms for a circular obstacle and inter-robot safety'; ...
    'U20', 'switching_geometric_graph', 'updates the graph from robot positions and monitors lambda_2(L(t))'; ...
    'U21', 'edge_dependent_delays', 'assigns a different communication delay tau_ij to each active edge'; ...
    'U22', 'time_varying_delays', 'uses a sinusoidal delay tau(t), including a crossing-critical profile'; ...
    'U23', 'potential_field_edge_clearance', 'uses attraction/repulsion potentials to obtain nonzero inter-agent clearance'; ...
    'U24', 'switching_obstacle_avoidance', 'switches from consensus to a local tangential avoidance controller when an obstacle is detected'; ...
    'U25', 'initial_history_protocol', 'tests the effect of constant versus zero pre-history on delayed unicycle consensus' ...
};
write_cell_csv_local(fullfile(TABLE_DIR, 'u26_extension_summary.csv'), ...
    {'experiment','extension','what_it_tests'}, extension_summary_rows);

% ============================================================
% Final summary
% ============================================================
fprintf('\n========== DONE ==========%s', newline);
fprintf('Generated tables in:  %s\n', TABLE_DIR);
fprintf('Generated figures in: %s\n', FIG_DIR);
if MAKE_VIDEOS
    fprintf('Generated videos in:  %s\n', VIDEO_DIR);
else
    fprintf('Video generation skipped. Set MAKE_VIDEOS = true to enable it.\n');
end

end  % end controller loop

end  % end main

% ============================================================
% Local helper functions
% ============================================================

function [time, states, commands, diag_data] = simulate_unicycle_delay_model_local(A, states0, model, tau, T, dt, params)
    N = size(states0, 1);
    steps = round(T / dt);
    time = (0:steps)' * dt;
    states = zeros(steps + 1, N, 3);
    commands = zeros(steps + 1, N, 2);
    states(1,:,:) = states0;

    D = diag(sum(A, 2));
    L = D - A;

    for k = 1:steps
        t = time(k);
        P_current = squeeze(states(k,:,1:2));
        if tau <= 0
            P_delay = P_current;
        else
            P_delay = delayed_positions_local(states, time, k, t - tau, states0);
        end

        if strcmp(model, 'full_state')
            U = -params.k * (L * P_delay);
        elseif strcmp(model, 'neighbor_only')
            U = -params.k * (D * P_current - A * P_delay);
        else
            error('Unknown delay model: %s', model);
        end

        if isfield(params, 'controller_type') && strcmp(params.controller_type, 'paper')
            if strcmp(model, 'full_state')
                P_self_for_paper = P_delay;
            else
                P_self_for_paper = P_current;
            end
            [states, commands] = integrate_paper_consensus_step_local( ...
                states, commands, k, dt, A, P_self_for_paper, P_delay, params);
        else
            for i = 1:N
                theta = states(k, i, 3);
                u_i = U(i, :);
                [v_i, omega_i] = cartesian_velocity_to_unicycle_local(u_i, theta, params);

                commands(k, i, 1) = v_i;
                commands(k, i, 2) = omega_i;

                states(k+1, i, 1) = states(k, i, 1) + dt * v_i * cos(theta);
                states(k+1, i, 2) = states(k, i, 2) + dt * v_i * sin(theta);
                states(k+1, i, 3) = wrap_to_pi_local(theta + dt * omega_i);
            end
        end
    end

    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
end

function [states, commands] = integrate_paper_consensus_step_local(states, commands, k, dt, A, P_self, P_neighbor, params)
    % Direct rendezvous controller inspired by Listmann et al.
    N = size(states, 2);
    for i = 1:N
        theta = states(k, i, 3);
        tangent = [cos(theta), sin(theta)];
        v_i = 0;
        omega_i = 0;
        neighbors = find(A(i,:) > 0);
        for idx = 1:length(neighbors)
            j = neighbors(idx);
            aij = A(i,j);
            dij = P_self(i,:) - P_neighbor(j,:);
            v_i = v_i - params.k * aij * (dij * tangent');
            if norm(dij) > params.velocity_epsilon
                phi_ij = atan2(P_neighbor(j,2) - P_self(i,2), P_neighbor(j,1) - P_self(i,1));
                omega_i = omega_i - params.paper_omega_gain * aij * wrap_to_pi_local(theta - phi_ij);
            end
        end
        if params.saturate
            v_i = clip_scalar_local(v_i, -params.vmax, params.vmax);
            omega_i = clip_scalar_local(omega_i, -params.omegamax, params.omegamax);
        end
        commands(k, i, 1) = v_i;
        commands(k, i, 2) = omega_i;
        states(k+1, i, 1) = states(k, i, 1) + dt * v_i * cos(theta);
        states(k+1, i, 2) = states(k, i, 2) + dt * v_i * sin(theta);
        states(k+1, i, 3) = wrap_to_pi_local(theta + dt * omega_i);
    end
end

function P_delay = delayed_positions_local(states, time, k, query_t, states0)
    if query_t <= 0
        P_delay = states0(:, 1:2);
        return;
    end

    dt = time(2) - time(1);
    idx0 = floor(query_t / dt) + 1;
    idx0 = max(1, min(idx0, k));
    idx1 = min(idx0 + 1, k);

    t0 = time(idx0);
    if idx1 == idx0
        alpha = 0;
    else
        alpha = (query_t - t0) / dt;
        alpha = max(0, min(1, alpha));
    end

    P0 = squeeze(states(idx0,:,1:2));
    P1 = squeeze(states(idx1,:,1:2));
    P_delay = (1 - alpha) * P0 + alpha * P1;
end

function [v, omega] = cartesian_velocity_to_unicycle_local(u, theta, params)
    speed_ref = norm(u);
    if speed_ref < params.velocity_epsilon
        v = 0;
        omega = 0;
        return;
    end

    heading = [cos(theta), sin(theta)];
    theta_ref = atan2(u(2), u(1));
    heading_error = wrap_to_pi_local(theta_ref - theta);

    if isfield(params, 'controller_type') && strcmp(params.controller_type, 'paper')
        v = params.kv * (u * heading');
        omega = params.ktheta * heading_error;
    else
        if params.forward_only
            alignment = max(0, cos(heading_error));
        else
            alignment = cos(heading_error);
        end
        v = params.kv * speed_ref * alignment;
        omega = params.ktheta * heading_error;
    end

    if params.saturate
        v = clip_scalar_local(v, -params.vmax, params.vmax);
        omega = clip_scalar_local(omega, -params.omegamax, params.omegamax);
    end
end

function A = adjacency_matrix_local(graph_type, N)
    A = zeros(N);
    if strcmp(graph_type, 'path')
        for i = 1:N-1
            A(i, i+1) = 1;
            A(i+1, i) = 1;
        end
    elseif strcmp(graph_type, 'ring')
        for i = 1:N
            j = mod(i, N) + 1;
            A(i, j) = 1;
            A(j, i) = 1;
        end
    elseif strcmp(graph_type, 'star')
        for i = 2:N
            A(1, i) = 1;
            A(i, 1) = 1;
        end
    elseif strcmp(graph_type, 'complete')
        A = ones(N) - eye(N);
    elseif strcmp(graph_type, 'disconnected')
        A(1,2) = 1; A(2,1) = 1;
        A(3,4) = 1; A(4,3) = 1;
        A(5,6) = 1; A(6,5) = 1;
    else
        error('Unknown graph type: %s', graph_type);
    end
end

function L = laplacian_matrix_local(A)
    D = diag(sum(A, 2));
    L = D - A;
end

function info = graph_spectral_info_local(L, k_gain)
    eigvals = sort(real(eig(L)));
    tol = 1e-8;
    positive = eigvals(eigvals > tol);
    if isempty(positive)
        lambda_2 = 0;
    else
        lambda_2 = positive(1);
    end
    lambda_max = max(eigvals);
    is_connected = (lambda_2 > tol);
    if lambda_max > tol
        tau_crit = pi / (2 * k_gain * lambda_max);
    else
        tau_crit = Inf;
    end
    info.eigvals = eigvals;
    info.lambda_2 = lambda_2;
    info.lambda_max = lambda_max;
    info.tau_crit = tau_crit;
    info.is_connected = is_connected;
end

function A_w = apply_symmetric_random_weights_local(A, min_w, max_w, seed)
    safe_rng_local(seed);
    N = size(A, 1);
    A_w = zeros(N);
    for i = 1:N
        for j = i+1:N
            if A(i,j) ~= 0
                w = min_w + (max_w - min_w) * rand();
                A_w(i,j) = w;
                A_w(j,i) = w;
            end
        end
    end
    safe_rng_local(7);
end

function A = random_connected_graph_local(N, p_edge, max_attempts, seed)
    safe_rng_local(seed);
    for attempt = 1:max_attempts
        R = rand(N) < p_edge;
        A = triu(R, 1);
        A = A + A';
        if is_connected_graph_local(A)
            safe_rng_local(7);
            return;
        end
    end
    A = adjacency_matrix_local('ring', N);
    safe_rng_local(7);
end

function [A, pos] = random_geometric_graph_local(N, radius, box_half_width, max_attempts, seed)
    safe_rng_local(seed);
    for attempt = 1:max_attempts
        pos = -box_half_width + 2 * box_half_width * rand(N, 2);
        A = zeros(N);
        for i = 1:N
            for j = i+1:N
                if norm(pos(i,:) - pos(j,:)) <= radius
                    A(i,j) = 1;
                    A(j,i) = 1;
                end
            end
        end
        if is_connected_graph_local(A)
            safe_rng_local(7);
            return;
        end
    end
    A = adjacency_matrix_local('ring', N);
    pos = zeros(N, 2);
    safe_rng_local(7);
end

function connected = is_connected_graph_local(A)
    N = size(A, 1);
    visited = false(N, 1);
    queue = zeros(N, 1);
    head = 1;
    tail = 1;
    queue(tail) = 1;
    visited(1) = true;
    while head <= tail
        node = queue(head);
        head = head + 1;
        neighbors = find(A(node,:) > 0);
        for idx = 1:length(neighbors)
            nb = neighbors(idx);
            if ~visited(nb)
                tail = tail + 1;
                queue(tail) = nb;
                visited(nb) = true;
            end
        end
    end
    connected = all(visited);
end

function d = disagreement_norm_2d_local(P_series)
    steps = size(P_series, 1);
    d = zeros(steps, 1);
    for k = 1:steps
        P = squeeze(P_series(k,:,:));
        c = mean(P, 1);
        diff = P - repmat(c, size(P,1), 1);
        d(k) = sqrt(mean(sum(diff.^2, 2)));
    end
end

function C = centroid_time_series_local(P_series)
    steps = size(P_series, 1);
    C = zeros(steps, 2);
    for k = 1:steps
        P = squeeze(P_series(k,:,:));
        C(k,:) = mean(P, 1);
    end
end

function summary = summarize_unicycle_run_local(time, states, commands, eps_convergence)
    P_series = states(:,:,1:2);
    d = disagreement_norm_2d_local(P_series);
    summary.final_disagreement = d(end);
    summary.max_disagreement = max(d);
    summary.convergence_time = convergence_time_metric_local(time, d, eps_convergence);
    final_positions = squeeze(states(end,:,1:2));
    summary.final_centroid = mean(final_positions, 1);
    summary.mean_path_length = mean_path_length_local(states);
    summary.max_abs_v = max(max(abs(squeeze(commands(:,:,1)))));
    summary.max_abs_omega = max(max(abs(squeeze(commands(:,:,2)))));
end

function tc = convergence_time_metric_local(time, d, eps_convergence)
    tc = NaN;
    for idx = 1:length(d)
        if all(d(idx:end) <= eps_convergence)
            tc = time(idx);
            return;
        end
    end
end

function stable_flag = practical_stability_flag_local(time, d)
    n = length(d);
    tail_start = max(1, round(0.8 * n));
    t_tail = time(tail_start:end);
    d_tail = d(tail_start:end);
    if length(d_tail) < 3
        stable_flag = false;
        return;
    end
    p = polyfit(t_tail(:), d_tail(:), 1);
    tail_slope = p(1);
    stable_flag = (d(end) < 0.15 && tail_slope <= 1e-3) || (d(end) < 1e-2);
end

function Lmean = mean_path_length_local(states)
    steps = size(states, 1);
    N = size(states, 2);
    lengths = zeros(N, 1);
    for i = 1:N
        P = squeeze(states(:, i, 1:2));
        diffs = diff(P, 1, 1);
        lengths(i) = sum(sqrt(sum(diffs.^2, 2)));
    end
    Lmean = mean(lengths);
end

function print_summary_local(label, tau, taucrit, summary)
    fprintf('%s\n', label);
    fprintf('  tau = %.6f, tau/taucrit = %.3f\n', tau, tau/taucrit);
    fprintf('  final disagreement = %.6e\n', summary.final_disagreement);
    fprintf('  max disagreement   = %.6e\n', summary.max_disagreement);
    fprintf('  convergence time   = %.6f\n', summary.convergence_time);
    fprintf('  final centroid     = [%.4f, %.4f]\n', summary.final_centroid(1), summary.final_centroid(2));
    fprintf('  mean path length   = %.4f\n', summary.mean_path_length);
end

function ensure_dir_local(pathname)
    if ~exist(pathname, 'dir')
        mkdir(pathname);
    end
end

function write_cell_csv_local(filename, header, rows)
    fid = fopen(filename, 'w');
    if fid < 0
        error('Could not open CSV file: %s', filename);
    end

    write_csv_row_local(fid, header);

    if isempty(rows)
        fclose(fid);
        return;
    end

    if ~iscell(rows)
        rows = num2cell(rows);
    end

    if isvector(rows) && length(rows) == length(header)
        write_csv_row_local(fid, rows);
    else
        for r = 1:size(rows, 1)
            write_csv_row_local(fid, rows(r, :));
        end
    end
    fclose(fid);
end

function write_csv_row_local(fid, row)
    for c = 1:length(row)
        value = row{c};
        if isnumeric(value)
            if isscalar(value)
                text = sprintf('%.12g', value);
            else
                text = mat2str(value);
            end
        elseif islogical(value)
            text = sprintf('%d', value);
        elseif ischar(value)
            text = value;
        else
            text = char(value);
        end
        text = strrep(text, '"', '""');
        fprintf(fid, '"%s"', text);
        if c < length(row)
            fprintf(fid, ',');
        end
    end
    fprintf(fid, '\n');
end

function plot_unicycle_trajectories_local(states, plot_title, target_point)
    figure;
    hold on;
    grid on;
    axis equal;
    N = size(states, 2);
    for i = 1:N
        x = squeeze(states(:, i, 1));
        y = squeeze(states(:, i, 2));
        theta0 = states(1, i, 3);
        thetaf = states(end, i, 3);
        plot(x, y, 'LineWidth', 1.3);
        plot(x(1), y(1), 'o', 'MarkerSize', 6, 'LineWidth', 1.0);
        plot(x(end), y(end), 'x', 'MarkerSize', 8, 'LineWidth', 1.5);
        quiver(x(1), y(1), 0.35*cos(theta0), 0.35*sin(theta0), 0, 'LineWidth', 1.0);
        quiver(x(end), y(end), 0.35*cos(thetaf), 0.35*sin(thetaf), 0, 'LineWidth', 1.0);
    end
    if ~isempty(target_point)
        plot(target_point(1), target_point(2), 'p', 'MarkerSize', 14, 'LineWidth', 1.8);
    end
    xlabel('x [m]');
    ylabel('y [m]');
    title(plot_title, 'Interpreter', 'none');
end

function plot_disagreement_curves_local(time, curves, labels, plot_title)
    figure;
    hold on;
    grid on;
    if isvector(curves)
        plot(time, curves(:), 'LineWidth', 1.4);
    else
        for idx = 1:size(curves, 2)
            plot(time, curves(:, idx), 'LineWidth', 1.4);
        end
    end
    xlabel('time [s]');
    ylabel('RMS disagreement [m]');
    title(plot_title, 'Interpreter', 'none');
    legend(labels, 'Location', 'best');
end

function plot_graph_custom_local(A, plot_title)
    N = size(A, 1);
    angles = linspace(0, 2*pi, N+1)';
    angles(end) = [];
    pos = [cos(angles), sin(angles)];
    figure;
    hold on;
    grid on;
    axis equal;
    for i = 1:N
        for j = i+1:N
            if A(i,j) > 0
                plot([pos(i,1), pos(j,1)], [pos(i,2), pos(j,2)], 'LineWidth', 1 + A(i,j));
                if abs(A(i,j) - 1) > 1e-9
                    mid = 0.5 * (pos(i,:) + pos(j,:));
                    text(mid(1), mid(2), sprintf('%.2f', A(i,j)), 'FontSize', 8);
                end
            end
        end
    end
    plot(pos(:,1), pos(:,2), 'o', 'MarkerSize', 10, 'LineWidth', 1.5);
    for i = 1:N
        text(pos(i,1) + 0.06, pos(i,2) + 0.06, sprintf('%d', i));
    end
    title(plot_title, 'Interpreter', 'none');
    axis off;
end

function plot_tau_sweep_summary_local(rows, plot_title)
    full_x = [];
    full_y = [];
    neigh_x = [];
    neigh_y = [];
    for r = 1:size(rows, 1)
        model = rows{r, 1};
        mult = rows{r, 2};
        final_d = rows{r, 5};
        if strcmp(model, 'full_state')
            full_x(end+1) = mult;
            full_y(end+1) = final_d;
        else
            neigh_x(end+1) = mult;
            neigh_y(end+1) = final_d;
        end
    end
    figure;
    hold on;
    grid on;
    semilogy(full_x, full_y, '-o', 'LineWidth', 1.4);
    semilogy(neigh_x, neigh_y, '-s', 'LineWidth', 1.4);
    yl = ylim;
    plot([1.0, 1.0], yl, '--', 'LineWidth', 1.0);
    ylim(yl);
    xlabel('tau / taucrit(full-state reference)');
    ylabel('final RMS disagreement [m]');
    title(plot_title, 'Interpreter', 'none');
    legend({'full-state','neighbor-only'}, 'Location', 'best');
end

function plot_topology_summary_local(rows, plot_title)
    names = {};
    full_tc = [];
    neigh_tc = [];
    for r = 1:size(rows, 1)
        graph_name = rows{r, 1};
        model = rows{r, 2};
        tc = rows{r, 11};
        if strcmp(model, 'full_state')
            names{end+1} = graph_name;
            full_tc(end+1) = tc;
        else
            neigh_tc(end+1) = tc;
        end
    end
    figure;
    bar([full_tc(:), neigh_tc(:)]);
    grid on;
    set(gca, 'XTick', 1:length(names), 'XTickLabel', names);
    ylabel('convergence time [s]');
    title(plot_title, 'Interpreter', 'none');
    legend({'full-state','neighbor-only'}, 'Location', 'best');
end

function plot_random_graph_summary_local(rows, plot_title)
    data = cell2mat(rows(:, 2:end));
    lambda2 = data(:, 2);
    tc = data(:, 9);
    figure;
    scatter(lambda2, tc, 45, 'filled');
    grid on;
    xlabel('lambda_2(L)');
    ylabel('convergence time [s]');
    title(plot_title, 'Interpreter', 'none');
end

function plot_geometric_summary_local(rows, radii, plot_title)
    data = cell2mat(rows);
    figure;
    hold on;
    grid on;
    for ridx = 1:length(radii)
        mask = abs(data(:,1) - radii(ridx)) < 1e-9;
        scatter(data(mask,3), data(mask,11), 45, 'filled');
    end
    xlabel('number of edges');
    ylabel('convergence time [s]');
    title(plot_title, 'Interpreter', 'none');
    legend({'R=4','R=5','R=6'}, 'Location', 'best');
end

function plot_dt_summary_local(rows, plot_title)
    data = cell2mat(rows);
    figure;
    semilogx(data(:,1), data(:,4), '-o', 'LineWidth', 1.4);
    grid on;
    xlabel('integration step dt [s]');
    ylabel('final RMS disagreement [m]');
    title(plot_title, 'Interpreter', 'none');
end

function plot_gain_summary_local(rows, plot_title)
    data = cell2mat(rows);
    figure;
    hold on;
    grid on;
    plot(data(:,1), data(:,3), '-o', 'LineWidth', 1.4);
    plot(data(:,1), data(:,6), '-s', 'LineWidth', 1.4);
    xlabel('consensus gain k');
    ylabel('value');
    title(plot_title, 'Interpreter', 'none');
    legend({'taucrit [s]','final RMS disagreement [m]'}, 'Location', 'best');
end

function plot_heading_summary_local(rows, plot_title)
    labels = rows(:,1);
    vals = cell2mat(rows(:,6));
    figure;
    bar(vals);
    grid on;
    set(gca, 'XTick', 1:length(labels), 'XTickLabel', labels);
    ylabel('convergence time [s]');
    title(plot_title, 'Interpreter', 'none');
end

function save_current_figure_local(filename, close_after_save)
    try
        saveas(gcf, filename);
    catch
        fprintf('Warning: could not save figure %s\n', filename);
    end
    if close_after_save
        close(gcf);
    end
end

% ============================================================
% ANIMAZIONE VIDEO (MATLAB 2025, sfondo bianco, skip se gia esiste)
% ============================================================
function save_unicycle_animation_local(time, states, filename, plot_title)
    % Genera l'animazione dei robot unicycle come file MP4.
    %
    % SKIP AUTOMATICO:
    %   se il video richiesto (o la sua eventuale versione .avi di fallback)
    %   esiste gia su disco, la funzione esce subito senza rigenerarlo.
    %   Questo permette di rilanciare il main senza ricalcolare i video
    %   gia prodotti.

    [folder, stem, ~] = fileparts(filename);
    if isempty(folder)
        folder = '.';
    end
    ensure_dir_local(folder);

    % --- Controllo "skip se gia generato" ---
    avi_fallback = fullfile(folder, [stem '.avi']);
    if exist(filename, 'file') == 2
        fprintf('Video gia presente, skip: %s\n', filename);
        return;
    end
    if exist(avi_fallback, 'file') == 2
        fprintf('Video gia presente (AVI), skip: %s\n', avi_fallback);
        return;
    end

    fps = 15;
    N = size(states, 2);
    skip = max(1, round(length(time) / 250));
    frame_idx = 1:skip:length(time);

    all_x = states(:,:,1);
    all_y = states(:,:,2);
    pad = 0.5;
    xlims = [min(all_x(:)) - pad, max(all_x(:)) + pad];
    ylims = [min(all_y(:)) - pad, max(all_y(:)) + pad];

    % --- Apertura VideoWriter (MPEG-4, con fallback AVI) ---
    writer = [];
    try
        writer = VideoWriter(filename, 'MPEG-4');
    catch
        try
            writer = VideoWriter(avi_fallback, 'Motion JPEG AVI');
            fprintf('MPEG-4 non disponibile, scrivo AVI: %s\n', avi_fallback);
        catch
            writer = [];
        end
    end

    if isempty(writer)
        fprintf('Impossibile creare il video: %s\n', filename);
        return;
    end

    writer.FrameRate = fps;
    open(writer);
    fig = make_white_figure_local();
    colors = lines(N);

    % La dimensione del primo frame catturato diventa il riferimento per
    % tutti i frame successivi. getframe puo restituire dimensioni in pixel
    % leggermente diverse (HiDPI, scaling, ridisegni) tra un frame e l'altro:
    % VideoWriter pero pretende frame della stessa identica dimensione. Per
    % questo ogni frame viene riportato (con imresize) alla dimensione del
    % primo, evitando l'errore "Frame must be H by W".
    ref_size = [];
    write_failed = false;
    for fk = 1:length(frame_idx)
        k = frame_idx(fk);
        draw_video_frame_local(fig, states, k, N, colors, xlims, ylims, plot_title, time(k));
        drawnow;
        frame = getframe(fig);
        img = frame.cdata;
        if isempty(ref_size)
            ref_size = [size(img, 1), size(img, 2)];
        elseif size(img, 1) ~= ref_size(1) || size(img, 2) ~= ref_size(2)
            img = imresize(img, ref_size);
        end
        try
            writeVideo(writer, img);
        catch write_err
            fprintf('Warning: scrittura frame fallita (%s). Salto il video: %s\n', ...
                write_err.message, filename);
            write_failed = true;
            break;
        end
    end
    close(fig);
    close(writer);
    if write_failed
        % Rimuove il file parziale/corrotto cosi un rilancio potra riprovare.
        if exist(writer.Filename, 'file') == 2
            try, delete(writer.Filename); catch, end
        end
        return;
    end
    fprintf('Video scritto: %s\n', filename);
end

function fig = make_white_figure_local()
    % Figura a dimensione fissa (in pixel) e non ridimensionabile: questo
    % rende stabile la dimensione restituita da getframe tra i frame.
    fig = figure('Visible', 'off', 'Color', 'w', ...
        'Units', 'pixels', 'Position', [100 100 640 480], ...
        'Resize', 'off');
    try
        set(fig, 'InvertHardcopy', 'off');
    catch
    end
    try
        set(fig, 'GraphicsSmoothing', 'on');
    catch
    end
end

function draw_video_frame_local(fig, states, k, N, colors, xlims, ylims, plot_title, t)
    clf(fig);
    ax = axes('Parent', fig, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, xlims);
    ylim(ax, ylims);
    for i = 1:N
        x = squeeze(states(1:k, i, 1));
        y = squeeze(states(1:k, i, 2));
        plot(ax, x, y, 'LineWidth', 1.0, 'Color', colors(i,:));
        theta = states(k, i, 3);
        quiver(ax, states(k,i,1), states(k,i,2), ...
            0.35*cos(theta), 0.35*sin(theta), 0, ...
            'LineWidth', 1.0, 'Color', colors(i,:), 'MaxHeadSize', 2);
        plot(ax, states(k,i,1), states(k,i,2), 'o', ...
            'MarkerSize', 5, 'MarkerFaceColor', colors(i,:), 'Color', colors(i,:));
    end
    title(ax, sprintf('%s, t = %.2f s', plot_title, t), 'Interpreter', 'none');
    xlabel(ax, 'x [m]');
    ylabel(ax, 'y [m]');
end

function y = wrap_to_pi_local(x)
    y = mod(x + pi, 2*pi) - pi;
end

function y = clip_scalar_local(x, lo, hi)
    y = min(max(x, lo), hi);
end

function states0 = generate_initial_unicycle_states_local(N, seed)
    if N == 6
        states0 = [ ...
            -3.0,  2.0,  0.2; ...
            -1.5, -2.0,  1.2; ...
             0.5,  3.0, -0.5; ...
             3.0, -1.0,  2.5; ...
             4.0,  1.5, -2.0; ...
            -2.0, -3.0,  0.8];
        return;
    end
    safe_rng_local(seed);
    angles = linspace(0, 2*pi, N+1)';
    angles(end) = [];
    radius = 3.0 + 0.15 * N;
    jitter = 0.25 * randn(N, 2);
    positions = radius * [cos(angles), sin(angles)] + jitter;
    headings = wrap_to_pi_local(angles + pi/2 + 0.35 * randn(N, 1));
    states0 = [positions, headings];
    safe_rng_local(7);
end

function U = base_consensus_velocity_local(A, states, time, k, states0, model, tau, params)
    P_current = squeeze(states(k,:,1:2));
    if tau <= 0
        P_delay = P_current;
    else
        P_delay = delayed_positions_local(states, time, k, time(k) - tau, states0);
    end
    D = diag(sum(A, 2));
    L = D - A;
    if strcmp(model, 'full_state')
        U = -params.k * (L * P_delay);
    elseif strcmp(model, 'neighbor_only')
        U = -params.k * (D * P_current - A * P_delay);
    else
        error('Unknown delay model: %s', model);
    end
end

function [time, states, commands, diag_data] = simulate_unicycle_target_rendezvous_local(A, states0, model, tau, T, dt, params, target_point, k_target)
    [time, states, commands] = initialize_unicycle_sim_local(states0, T, dt);
    steps = length(time) - 1;
    N = size(states0, 1);
    target_error = zeros(steps + 1, 1);

    for k = 1:steps
        P_current = squeeze(states(k,:,1:2));
        U = base_consensus_velocity_local(A, states, time, k, states0, model, tau, params);
        U = U + k_target * (repmat(target_point, N, 1) - P_current);
        [states, commands] = integrate_unicycle_step_local(states, commands, k, dt, U, params);
        target_error(k) = mean_distance_to_point_local(P_current, target_point);
    end
    target_error(end) = mean_distance_to_point_local(squeeze(states(end,:,1:2)), target_point);
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
    diag_data.target_error = target_error;
end

function [time, states, commands, diag_data] = simulate_unicycle_leader_follower_local(A, states0, model, tau, T, dt, params, leader_index, leader_fun, k_leader)
    [time, states, commands] = initialize_unicycle_sim_local(states0, T, dt);
    steps = length(time) - 1;
    N = size(states0, 1);
    leader_target = zeros(steps + 1, 2);
    leader_tracking_error = zeros(steps + 1, 1);
    follower_spread = zeros(steps + 1, 1);

    for k = 1:steps
        t = time(k);
        P_current = squeeze(states(k,:,1:2));
        U = base_consensus_velocity_local(A, states, time, k, states0, model, tau, params);
        target = leader_fun(t);
        U(leader_index,:) = k_leader * (target - P_current(leader_index,:));

        [states, commands] = integrate_unicycle_step_local(states, commands, k, dt, U, params);
        leader_target(k,:) = target;
        leader_tracking_error(k) = norm(P_current(leader_index,:) - target);
        followers = setdiff(1:N, leader_index);
        follower_spread(k) = disagreement_single_snapshot_local(P_current(followers,:));
    end
    leader_target(end,:) = leader_fun(time(end));
    P_final = squeeze(states(end,:,1:2));
    leader_tracking_error(end) = norm(P_final(leader_index,:) - leader_target(end,:));
    followers = setdiff(1:N, leader_index);
    follower_spread(end) = disagreement_single_snapshot_local(P_final(followers,:));
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
    diag_data.leader_target = leader_target;
    diag_data.leader_tracking_error = leader_tracking_error;
    diag_data.follower_spread = follower_spread;
end

function [time, states, commands, diag_data] = simulate_unicycle_formation_local(A, states0, model, tau, T, dt, params, offsets, formation_center, k_anchor)
    [time, states, commands] = initialize_unicycle_sim_local(states0, T, dt);
    steps = length(time) - 1;
    N = size(states0, 1);
    formation_error = zeros(steps + 1, 1);
    D = diag(sum(A, 2));
    L = D - A;

    for k = 1:steps
        P_current = squeeze(states(k,:,1:2));
        if tau <= 0
            P_delay = P_current;
        else
            P_delay = delayed_positions_local(states, time, k, time(k) - tau, states0);
        end
        Q_current = P_current - offsets;
        Q_delay = P_delay - offsets;
        if strcmp(model, 'full_state')
            U = -params.k * (L * Q_delay);
        elseif strcmp(model, 'neighbor_only')
            U = -params.k * (D * Q_current - A * Q_delay);
        else
            error('Unknown delay model: %s', model);
        end
        U = U + k_anchor * (repmat(formation_center, N, 1) - Q_current);
        [states, commands] = integrate_unicycle_step_local(states, commands, k, dt, U, params);
        formation_error(k) = formation_error_snapshot_local(P_current, offsets);
    end
    formation_error(end) = formation_error_snapshot_local(squeeze(states(end,:,1:2)), offsets);
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
    diag_data.formation_error = formation_error;
end

function [time, states, commands, diag_data] = simulate_unicycle_noise_local(A, states0, model, tau, T, dt, params, noise_std, seed)
    safe_rng_local(seed);
    [time, states, commands] = initialize_unicycle_sim_local(states0, T, dt);
    steps = length(time) - 1;
    N = size(states0, 1);
    D = diag(sum(A, 2));
    L = D - A;

    for k = 1:steps
        P_current = squeeze(states(k,:,1:2));
        if tau <= 0
            P_delay = P_current;
        else
            P_delay = delayed_positions_local(states, time, k, time(k) - tau, states0);
        end
        P_current_meas = P_current + noise_std * randn(N, 2);
        P_delay_meas = P_delay + noise_std * randn(N, 2);
        if strcmp(model, 'full_state')
            U = -params.k * (L * P_delay_meas);
        elseif strcmp(model, 'neighbor_only')
            U = -params.k * (D * P_current_meas - A * P_delay_meas);
        else
            error('Unknown delay model: %s', model);
        end
        [states, commands] = integrate_unicycle_step_local(states, commands, k, dt, U, params);
        for i = 1:N
            commands(k,i,1) = commands(k,i,1) + 0.35 * noise_std * randn();
            commands(k,i,2) = commands(k,i,2) + 0.70 * noise_std * randn();
            states(k+1,i,1) = states(k+1,i,1) + sqrt(dt) * 0.25 * noise_std * randn();
            states(k+1,i,2) = states(k+1,i,2) + sqrt(dt) * 0.25 * noise_std * randn();
            states(k+1,i,3) = wrap_to_pi_local(states(k+1,i,3) + sqrt(dt) * 0.15 * noise_std * randn());
        end
    end
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
    safe_rng_local(7);
end

function [time, states, commands, diag_data] = simulate_unicycle_packet_loss_local(A, states0, model, tau, T, dt, params, loss_prob, seed)
    safe_rng_local(seed);
    [time, states, commands] = initialize_unicycle_sim_local(states0, T, dt);
    steps = length(time) - 1;
    N = size(states0, 1);

    for k = 1:steps
        P_current = squeeze(states(k,:,1:2));
        if tau <= 0
            P_delay = P_current;
        else
            P_delay = delayed_positions_local(states, time, k, time(k) - tau, states0);
        end
        keep = rand(N) > loss_prob;
        keep = triu(keep, 1);
        keep = keep + keep';
        A_eff = A .* keep;
        D_eff = diag(sum(A_eff, 2));
        L_eff = D_eff - A_eff;
        if strcmp(model, 'full_state')
            U = -params.k * (L_eff * P_delay);
        elseif strcmp(model, 'neighbor_only')
            U = -params.k * (D_eff * P_current - A_eff * P_delay);
        else
            error('Unknown delay model: %s', model);
        end
        [states, commands] = integrate_unicycle_step_local(states, commands, k, dt, U, params);
    end
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
    safe_rng_local(7);
end

function [time, states, commands, diag_data] = simulate_unicycle_avoidance_local(A, states0, model, tau, T, dt, params, obstacle, safety)
    [time, states, commands] = initialize_unicycle_sim_local(states0, T, dt);
    steps = length(time) - 1;
    min_pairwise_distance = zeros(steps + 1, 1);
    obstacle_clearance = zeros(steps + 1, 1);

    for k = 1:steps
        P_current = squeeze(states(k,:,1:2));
        U = base_consensus_velocity_local(A, states, time, k, states0, model, tau, params);
        U = U + obstacle_repulsion_local(P_current, obstacle);
        U = U + inter_robot_repulsion_local(P_current, safety);
        [states, commands] = integrate_unicycle_step_local(states, commands, k, dt, U, params);
        min_pairwise_distance(k) = min_pairwise_distance_snapshot_local(P_current);
        obstacle_clearance(k) = min(sqrt(sum((P_current - repmat(obstacle.center, size(P_current,1), 1)).^2, 2)) - obstacle.radius);
    end
    P_final = squeeze(states(end,:,1:2));
    min_pairwise_distance(end) = min_pairwise_distance_snapshot_local(P_final);
    obstacle_clearance(end) = min(sqrt(sum((P_final - repmat(obstacle.center, size(P_final,1), 1)).^2, 2)) - obstacle.radius);
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
    diag_data.min_pairwise_distance = min_pairwise_distance;
    diag_data.obstacle_clearance = obstacle_clearance;
end

function [time, states, commands, diag_data] = simulate_unicycle_switching_graph_local(states0, model, tau, T, dt, params, comm_radius)
    [time, states, commands] = initialize_unicycle_sim_local(states0, T, dt);
    steps = length(time) - 1;
    N = size(states0, 1);
    lambda2_series = zeros(steps + 1, 1);
    num_edges_series = zeros(steps + 1, 1);

    for k = 1:steps
        P_current = squeeze(states(k,:,1:2));
        A_t = adjacency_from_positions_local(P_current, comm_radius);
        L_t = laplacian_matrix_local(A_t);
        info_t = graph_spectral_info_local(L_t, params.k);
        lambda2_series(k) = info_t.lambda_2;
        num_edges_series(k) = sum(A_t(:))/2;

        if tau <= 0
            P_delay = P_current;
        else
            P_delay = delayed_positions_local(states, time, k, time(k) - tau, states0);
        end
        D_t = diag(sum(A_t, 2));
        if strcmp(model, 'full_state')
            U = -params.k * (L_t * P_delay);
        elseif strcmp(model, 'neighbor_only')
            U = -params.k * (D_t * P_current - A_t * P_delay);
        else
            error('Unknown delay model: %s', model);
        end
        [states, commands] = integrate_unicycle_step_local(states, commands, k, dt, U, params);
    end
    P_final = squeeze(states(end,:,1:2));
    A_final = adjacency_from_positions_local(P_final, comm_radius);
    info_final = graph_spectral_info_local(laplacian_matrix_local(A_final), params.k);
    lambda2_series(end) = info_final.lambda_2;
    num_edges_series(end) = sum(A_final(:))/2;
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
    diag_data.lambda2 = lambda2_series;
    diag_data.num_edges = num_edges_series;
end

function [time, states, commands] = initialize_unicycle_sim_local(states0, T, dt)
    steps = round(T / dt);
    time = (0:steps)' * dt;
    N = size(states0, 1);
    states = zeros(steps + 1, N, 3);
    commands = zeros(steps + 1, N, 2);
    states(1,:,:) = states0;
end

function [states, commands] = integrate_unicycle_step_local(states, commands, k, dt, U, params)
    N = size(states, 2);
    for i = 1:N
        theta = states(k, i, 3);
        [v_i, omega_i] = cartesian_velocity_to_unicycle_local(U(i,:), theta, params);
        commands(k, i, 1) = v_i;
        commands(k, i, 2) = omega_i;
        states(k+1, i, 1) = states(k, i, 1) + dt * v_i * cos(theta);
        states(k+1, i, 2) = states(k, i, 2) + dt * v_i * sin(theta);
        states(k+1, i, 3) = wrap_to_pi_local(theta + dt * omega_i);
    end
end

function d = mean_distance_to_point_local(P, point)
    d = mean(sqrt(sum((P - repmat(point, size(P,1), 1)).^2, 2)));
end

function d = disagreement_single_snapshot_local(P)
    c = mean(P, 1);
    d = sqrt(mean(sum((P - repmat(c, size(P,1), 1)).^2, 2)));
end

function offsets = polygon_offsets_local(N, radius)
    angles = linspace(0, 2*pi, N+1)';
    angles(end) = [];
    offsets = radius * [cos(angles), sin(angles)];
end

function err = formation_error_snapshot_local(P, offsets)
    Q = P - offsets;
    err = disagreement_single_snapshot_local(Q);
end

function U_rep = obstacle_repulsion_local(P, obstacle)
    N = size(P, 1);
    U_rep = zeros(N, 2);
    for i = 1:N
        diff = P(i,:) - obstacle.center;
        dist = norm(diff);
        if dist < obstacle.influence_radius && dist > 1e-9
            clearance = max(dist - obstacle.radius, 1e-3);
            influence_clearance = obstacle.influence_radius - obstacle.radius;
            magnitude = obstacle.k_obstacle * (1/clearance - 1/influence_clearance) / (clearance^2);
            magnitude = max(0, min(magnitude, 3.0));
            U_rep(i,:) = magnitude * diff / dist;
        end
    end
end

function U_rep = inter_robot_repulsion_local(P, safety)
    N = size(P, 1);
    U_rep = zeros(N, 2);
    for i = 1:N
        for j = 1:N
            if i ~= j
                diff = P(i,:) - P(j,:);
                dist = norm(diff);
                if dist < safety.distance && dist > 1e-9
                    magnitude = safety.k_collision * (1/dist - 1/safety.distance) / (dist^2);
                    magnitude = max(0, min(magnitude, 2.5));
                    U_rep(i,:) = U_rep(i,:) + magnitude * diff / dist;
                end
            end
        end
    end
end

function dmin = min_pairwise_distance_snapshot_local(P)
    N = size(P, 1);
    dmin = Inf;
    for i = 1:N
        for j = i+1:N
            dmin = min(dmin, norm(P(i,:) - P(j,:)));
        end
    end
    if isinf(dmin)
        dmin = 0;
    end
end

function A = adjacency_from_positions_local(P, radius)
    N = size(P, 1);
    A = zeros(N);
    for i = 1:N
        for j = i+1:N
            if norm(P(i,:) - P(j,:)) <= radius
                A(i,j) = 1;
                A(j,i) = 1;
            end
        end
    end
end

function draw_circle_local(center, radius, line_style)
    angles = linspace(0, 2*pi, 200);
    x = center(1) + radius * cos(angles);
    y = center(2) + radius * sin(angles);
    plot(x, y, line_style, 'LineWidth', 1.2);
end

function plot_complete_graph_summary_local(rows, plot_title)
    numeric_rows = [];
    for r = 1:size(rows, 1)
        if strcmp(rows{r,3}, 'full_state') && abs(rows{r,8} - 0.50) < 1e-12
            numeric_rows(end+1, :) = [rows{r,1}, rows{r,2}, rows{r,6}, rows{r,7}, rows{r,10}];
        end
    end
    figure;
    hold on;
    grid on;
    weights = unique(numeric_rows(:,2));
    for widx = 1:length(weights)
        w = weights(widx);
        mask = abs(numeric_rows(:,2) - w) < 1e-12;
        plot(numeric_rows(mask,1), numeric_rows(mask,4), '-o', 'LineWidth', 1.4);
    end
    xlabel('complete graph size N');
    ylabel('taucrit [s]');
    title(plot_title, 'Interpreter', 'none');
    legend({'w=0.5','w=1.0','w=1.5'}, 'Location', 'best');
end

function plot_noise_summary_local(rows, plot_title)
    data = cell2mat(rows);
    figure;
    plot(data(:,1), data(:,4), '-o', 'LineWidth', 1.4);
    grid on;
    xlabel('noise standard deviation');
    ylabel('tail residual disagreement [m]');
    title(plot_title, 'Interpreter', 'none');
end

function plot_packet_loss_summary_local(rows, plot_title)
    data = cell2mat(rows);
    figure;
    plot(data(:,1), data(:,4), '-o', 'LineWidth', 1.4);
    grid on;
    xlabel('packet loss probability');
    ylabel('tail residual disagreement [m]');
    title(plot_title, 'Interpreter', 'none');
end

function plot_scalar_time_series_local(time, values, ylabel_text, plot_title)
    figure;
    plot(time, values, 'LineWidth', 1.4);
    grid on;
    xlabel('time [s]');
    ylabel(ylabel_text, 'Interpreter', 'none');
    title(plot_title, 'Interpreter', 'none');
end

function Tau = generate_edge_delay_matrix_local(A, taucrit, low_ratio, high_ratio, seed, symmetric)
    safe_rng_local(seed);
    N = size(A, 1);
    Tau = zeros(N);
    if symmetric
        for i = 1:N
            for j = i+1:N
                if A(i,j) > 0
                    val = (low_ratio + (high_ratio - low_ratio) * rand()) * taucrit;
                    Tau(i,j) = val;
                    Tau(j,i) = val;
                end
            end
        end
    else
        for i = 1:N
            for j = 1:N
                if A(i,j) > 0
                    Tau(i,j) = (low_ratio + (high_ratio - low_ratio) * rand()) * taucrit;
                end
            end
        end
    end
    safe_rng_local(7);
end

function rows = edge_delay_rows_local(Tau, A, case_name, taucrit)
    rows = {};
    N = size(A, 1);
    for i = 1:N
        for j = 1:N
            if A(i,j) > 0
                rows(end+1, :) = {case_name, i, j, Tau(i,j), Tau(i,j) / taucrit};
            end
        end
    end
end

function [time, states, commands, diag_data] = simulate_unicycle_edge_dependent_delays_local(A, states0, model, Tau_edges, T, dt, params)
    N = size(states0, 1);
    steps = round(T / dt);
    time = (0:steps)' * dt;
    states = zeros(steps + 1, N, 3);
    commands = zeros(steps + 1, N, 2);
    states(1,:,:) = states0;

    active_taus = Tau_edges(A > 0);
    tau_max = 0;
    tau_mean = 0;
    if ~isempty(active_taus)
        tau_max = max(active_taus);
        tau_mean = mean(active_taus);
    end
    diag_data.tau_max = tau_max * ones(steps + 1, 1);
    diag_data.tau_mean = tau_mean * ones(steps + 1, 1);

    for k = 1:steps
        t = time(k);
        P_current = squeeze(states(k,:,1:2));
        U = zeros(N, 2);
        for i = 1:N
            neighbors = find(A(i,:) > 0);
            for jj = 1:length(neighbors)
                j = neighbors(jj);
                tau_ij = Tau_edges(i,j);
                if tau_ij <= 0
                    P_delay = P_current;
                else
                    P_delay = delayed_positions_local(states, time, k, t - tau_ij, states0);
                end
                if strcmp(model, 'full_state')
                    U(i,:) = U(i,:) - params.k * A(i,j) * (P_delay(i,:) - P_delay(j,:));
                elseif strcmp(model, 'neighbor_only')
                    U(i,:) = U(i,:) - params.k * A(i,j) * (P_current(i,:) - P_delay(j,:));
                else
                    error('Unknown delay model: %s', model);
                end
            end
        end
        [states, commands] = integrate_unicycle_step_series_local(states, commands, k, U, dt, params);
    end
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
end

function [time, states, commands, diag_data] = simulate_unicycle_time_varying_delay_local(A, states0, model, tau_function, T, dt, params)
    N = size(states0, 1);
    steps = round(T / dt);
    time = (0:steps)' * dt;
    states = zeros(steps + 1, N, 3);
    commands = zeros(steps + 1, N, 2);
    states(1,:,:) = states0;
    tau_series = zeros(steps + 1, 1);

    for k = 1:steps
        t = time(k);
        tau_t = max(0, tau_function(t));
        tau_series(k) = tau_t;
        P_current = squeeze(states(k,:,1:2));
        if tau_t <= 0
            P_delay = P_current;
        else
            P_delay = delayed_positions_local(states, time, k, t - tau_t, states0);
        end
        U = consensus_velocity_local(A, P_current, P_delay, model, params);
        [states, commands] = integrate_unicycle_step_series_local(states, commands, k, U, dt, params);
    end
    tau_series(end) = max(0, tau_function(time(end)));
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
    diag_data.tau = tau_series;
end

function U = consensus_velocity_local(A, P_current, P_delay, model, params)
    D = diag(sum(A, 2));
    L = D - A;
    if strcmp(model, 'full_state')
        U = -params.k * (L * P_delay);
    elseif strcmp(model, 'neighbor_only')
        U = -params.k * (D * P_current - A * P_delay);
    else
        error('Unknown delay model: %s', model);
    end
end

function [states, commands] = integrate_unicycle_step_series_local(states, commands, k, U, dt, params)
    N = size(states, 2);
    for i = 1:N
        theta = states(k, i, 3);
        [v_i, omega_i] = cartesian_velocity_to_unicycle_local(U(i,:), theta, params);
        commands(k, i, 1) = v_i;
        commands(k, i, 2) = omega_i;
        states(k+1, i, 1) = states(k, i, 1) + dt * v_i * cos(theta);
        states(k+1, i, 2) = states(k, i, 2) + dt * v_i * sin(theta);
        states(k+1, i, 3) = wrap_to_pi_local(theta + dt * omega_i);
    end
end

function [time, states, commands, diag_data] = simulate_unicycle_potential_clearance_local(A, states0, model, tau, T, dt, params, desired_clearance, attraction_gain, anchor_point, anchor_gain)
    N = size(states0, 1);
    steps = round(T / dt);
    time = (0:steps)' * dt;
    states = zeros(steps + 1, N, 3);
    commands = zeros(steps + 1, N, 2);
    states(1,:,:) = states0;
    c = desired_clearance^2;

    for k = 1:steps
        t = time(k);
        P_current = squeeze(states(k,:,1:2));
        if tau <= 0
            P_delay = P_current;
        else
            P_delay = delayed_positions_local(states, time, k, t - tau, states0);
        end
        if strcmp(model, 'full_state')
            P_self = P_delay;
        else
            P_self = P_current;
        end
        U = zeros(N, 2);
        for i = 1:N
            for j = 1:N
                if A(i,j) <= 0
                    continue;
                end
                diff = P_self(i,:) - P_delay(j,:);
                r2 = diff * diff';
                kd = 1.0 - exp(max(-20.0, min(20.0, c - r2)));
                U(i,:) = U(i,:) - attraction_gain * A(i,j) * kd * diff;
            end
            if ~isempty(anchor_point) && anchor_gain > 0
                centroid = mean(P_current, 1);
                U(i,:) = U(i,:) + anchor_gain * (anchor_point - centroid);
            end
        end
        [states, commands] = integrate_unicycle_step_series_local(states, commands, k, U, dt, params);
    end
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.edge_distance_error = edge_distance_error_time_series_local(states(:,:,1:2), A, desired_clearance);
end

function vals = edge_distance_error_time_series_local(P_series, A, desired_clearance)
    steps = size(P_series, 1);
    vals = zeros(steps, 1);
    edges = [];
    for i = 1:size(A,1)
        for j = i+1:size(A,2)
            if A(i,j) > 0
                edges(end+1,:) = [i, j];
            end
        end
    end
    if isempty(edges)
        return;
    end
    for k = 1:steps
        P = squeeze(P_series(k,:,:));
        errs = zeros(size(edges,1), 1);
        for e = 1:size(edges,1)
            i = edges(e,1); j = edges(e,2);
            errs(e) = abs(norm(P(i,:) - P(j,:)) - desired_clearance);
        end
        vals(k) = mean(errs);
    end
end

function [time, states, commands, diag_data] = simulate_unicycle_switching_obstacle_controller_local(A, states0, model, tau, T, dt, params, obstacle, detection_radius, k_avoid)
    N = size(states0, 1);
    steps = round(T / dt);
    time = (0:steps)' * dt;
    states = zeros(steps + 1, N, 3);
    commands = zeros(steps + 1, N, 2);
    states(1,:,:) = states0;
    avoidance_active_count = zeros(steps + 1, 1);
    obstacle_clearance = zeros(steps + 1, 1);

    for k = 1:steps
        t = time(k);
        P_current = squeeze(states(k,:,1:2));
        if tau <= 0
            P_delay = P_current;
        else
            P_delay = delayed_positions_local(states, time, k, t - tau, states0);
        end
        U = consensus_velocity_local(A, P_current, P_delay, model, params);
        active_count = 0;
        for i = 1:N
            p = P_current(i,:);
            theta = states(k, i, 3);
            to_obstacle = obstacle.center - p;
            dist = norm(to_obstacle);
            heading = [cos(theta), sin(theta)];
            moving_towards_obstacle = (heading * to_obstacle') > 0;
            if dist <= detection_radius && moving_towards_obstacle
                e = to_obstacle / max(dist, 1e-9);
                tangent = [-e(2), e(1)];
                away = -e * max(0, obstacle.radius + 0.55 - dist);
                U(i,:) = k_avoid * tangent + 1.2 * away;
                active_count = active_count + 1;
            end
        end
        avoidance_active_count(k) = active_count;
        obstacle_clearance(k) = min(sqrt(sum((P_current - repmat(obstacle.center, N, 1)).^2, 2)) - obstacle.radius);
        [states, commands] = integrate_unicycle_step_series_local(states, commands, k, U, dt, params);
    end
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    P_final = squeeze(states(end,:,1:2));
    obstacle_clearance(end) = min(sqrt(sum((P_final - repmat(obstacle.center, N, 1)).^2, 2)) - obstacle.radius);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.obstacle_clearance = obstacle_clearance;
    diag_data.avoidance_active_count = avoidance_active_count;
end

function P_delay = delayed_positions_with_history_local(states, time, k, query_t, states0, history_mode)
    if query_t <= 0
        if strcmp(history_mode, 'zero_prehistory')
            P_delay = zeros(size(states0,1), 2);
        elseif strcmp(history_mode, 'constant')
            P_delay = states0(:,1:2);
        else
            error('Unknown history mode: %s', history_mode);
        end
        return;
    end
    P_delay = delayed_positions_local(states, time, k, query_t, states0);
end

function [time, states, commands, diag_data] = simulate_unicycle_delay_model_history_local(A, states0, model, tau, T, dt, params, history_mode)
    N = size(states0, 1);
    steps = round(T / dt);
    time = (0:steps)' * dt;
    states = zeros(steps + 1, N, 3);
    commands = zeros(steps + 1, N, 2);
    states(1,:,:) = states0;

    for k = 1:steps
        t = time(k);
        P_current = squeeze(states(k,:,1:2));
        if tau <= 0
            P_delay = P_current;
        else
            P_delay = delayed_positions_with_history_local(states, time, k, t - tau, states0, history_mode);
        end
        U = consensus_velocity_local(A, P_current, P_delay, model, params);
        [states, commands] = integrate_unicycle_step_series_local(states, commands, k, U, dt, params);
    end
    last_command_idx = max(1, size(commands, 1) - 1);
    commands(end,:,:) = commands(last_command_idx,:,:);
    diag_data.disagreement = disagreement_norm_2d_local(states(:,:,1:2));
    diag_data.centroid = centroid_time_series_local(states(:,:,1:2));
end

function plot_edge_delay_summary_local(rows, plot_title)
    labels = {};
    vals_full = [];
    vals_neigh = [];
    case_order = {};
    for r = 1:size(rows, 1)
        case_name = rows{r,1};
        if ~any(strcmp(case_order, case_name))
            case_order{end+1} = case_name;
        end
    end
    for c = 1:length(case_order)
        labels{end+1} = case_order{c};
        vf = NaN; vn = NaN;
        for r = 1:size(rows, 1)
            if strcmp(rows{r,1}, case_order{c}) && strcmp(rows{r,2}, 'full_state')
                vf = rows{r,11};
            elseif strcmp(rows{r,1}, case_order{c}) && strcmp(rows{r,2}, 'neighbor_only')
                vn = rows{r,11};
            end
        end
        vals_full(end+1) = vf;
        vals_neigh(end+1) = vn;
    end
    figure;
    bar([vals_full(:), vals_neigh(:)]);
    grid on;
    set(gca, 'XTick', 1:length(labels), 'XTickLabel', labels);
    ylabel('final RMS disagreement [m]');
    title(plot_title, 'Interpreter', 'none');
    legend({'full-state','neighbor-only'}, 'Location', 'best');
end

function plot_time_varying_summary_local(rows, plot_title)
    labels = {};
    vals_full = [];
    vals_neigh = [];
    case_order = {};
    for r = 1:size(rows, 1)
        case_name = rows{r,1};
        if ~any(strcmp(case_order, case_name))
            case_order{end+1} = case_name;
        end
    end
    for c = 1:length(case_order)
        labels{end+1} = case_order{c};
        vf = NaN; vn = NaN;
        for r = 1:size(rows, 1)
            if strcmp(rows{r,1}, case_order{c}) && strcmp(rows{r,2}, 'full_state')
                vf = rows{r,12};
            elseif strcmp(rows{r,1}, case_order{c}) && strcmp(rows{r,2}, 'neighbor_only')
                vn = rows{r,12};
            end
        end
        vals_full(end+1) = vf;
        vals_neigh(end+1) = vn;
    end
    figure;
    bar([vals_full(:), vals_neigh(:)]);
    grid on;
    set(gca, 'XTick', 1:length(labels), 'XTickLabel', labels);
    ylabel('final RMS disagreement [m]');
    title(plot_title, 'Interpreter', 'none');
    legend({'full-state','neighbor-only'}, 'Location', 'best');
end

function plot_history_summary_local(rows, plot_title)
    labels = {};
    vals = [];
    for r = 1:size(rows, 1)
        labels{end+1} = [rows{r,2} '-' rows{r,3}];
        vals(end+1) = rows{r,8};
    end
    figure;
    bar(vals(:));
    grid on;
    set(gca, 'XTick', 1:length(labels), 'XTickLabel', labels);
    ylabel('final RMS disagreement [m]');
    title(plot_title, 'Interpreter', 'none');
end

function safe_rng_local(seed)
    % Robust RNG initialization for MATLAB/Octave.
    try
        rng(seed, 'twister');
    catch
        try
            rand('state', seed);
            randn('state', seed);
        catch
        end
    end
end