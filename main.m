function D = weighted_degree_matrix(A)
%WEIGHTED_DEGREE_MATRIX Return the (diagonal) weighted degree matrix of A.
%   D = weighted_degree_matrix(A) returns a diagonal matrix D where
%   D(i,i) = sum_j A(i,j). Works for binary or weighted adjacency matrices.
%
%   Inputs:
%     A - square adjacency matrix (NxN)
%   Outputs:
%     D - NxN diagonal matrix of node degrees

% basic input validation
if ~ismatrix(A) || size(A,1) ~= size(A,2)
    error('weighted_degree_matrix:Input', 'A must be a square matrix.');
end

% ensure numeric
A = double(A);

% sum of each row (or column for symmetric graphs)
deg = sum(A, 2);

% build diagonal matrix
D = diag(deg);
end
% Control of Multi-Robot Systems - Project 2026
%
% Plain MATLAB/Octave implementation.
%
% This script runs the complete project:
%   1. 1D delayed consensus
%   2. 2D delayed consensus
%   3. delay comparison
%   4. topology comparison
%   5. random connected graphs
%   6. random geometric graphs
%   7. disconnected graph
%   8. integration-step sensitivity
%   9. unicycle rendezvous
%   10. saturated vs unsaturated unicycle control
%   11. weighted graphs
%   12. full-state delay vs neighbor-only delay
%   13. unicycle comparison for delay models
%   14. video generation

clear all;
close all;
clc;

% ============================================================
% Compatibility configuration
% ============================================================
% Set this to false if Octave gives graphics errors or if you only want tables.
MAKE_FIGURES = true;

% Save figures to disk and close them immediately instead of showing them.
CLOSE_FIGURES_AFTER_SAVE = true;

% Set this to false if video generation is slow or unsupported.
MAKE_VIDEOS = true;

% Octave sometimes needs an explicit graphics toolkit.
% If none is available, the code will still run with MAKE_FIGURES=false.
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

% Extra safeguard: keep figures invisible and save them to disk.
try
    set(0, 'DefaultFigureVisible', 'off');
catch
end

rng(7);

%% Output folders
OUTPUT_DIR = 'p1_outputs_matlab';
FIG_DIR = fullfile(OUTPUT_DIR, 'figures');
TABLE_DIR = fullfile(OUTPUT_DIR, 'tables');
VIDEO_DIR = fullfile(OUTPUT_DIR, 'videos');

ensure_dir(OUTPUT_DIR);
ensure_dir(FIG_DIR);
ensure_dir(TABLE_DIR);
ensure_dir(VIDEO_DIR);

fprintf('Output directory: %s\n\n', OUTPUT_DIR);

%% Initial setup
N = 6;

x0 = [-3.0; -1.5; 0.5; 3.0; 4.0; -2.0];

P0 = [...
    -3.0,  2.0;
    -1.5, -2.0;
     0.5,  3.0;
     3.0, -1.0;
     4.0,  1.5;
    -2.0, -3.0];

initial_unicycle_states = [...
    -3.0,  2.0,  0.2;
    -1.5, -2.0,  1.2;
     0.5,  3.0, -0.5;
     3.0, -1.0,  2.5;
     4.0,  1.5, -2.0;
    -2.0, -3.0,  0.8];

A_ring = adjacency_matrix('ring', N);
L_ring = laplacian_matrix(A_ring);
info_ring = graph_spectral_info(L_ring);

fprintf('========== GRAPH INFORMATION ==========\n');
fprintf('Graph type: ring\n');
disp('Adjacency matrix A:');
disp(A_ring);
disp('Laplacian matrix L:');
disp(L_ring);
disp('Laplacian eigenvalues:');
disp(info_ring.eigvals');
fprintf('lambda_2: %.12g\n', info_ring.lambda_2);
fprintf('lambda_max: %.12g\n', info_ring.lambda_max);
fprintf('tau_crit = pi/(2 lambda_max): %.12g\n', info_ring.tau_crit);
fprintf('connected: %d\n\n', info_ring.is_connected);

% Neighbor-only delay model stability note.
% For xdot = -D*x(t) + A*x(t-tau), there is no finite tau_crit
% analogous to pi/(2*lambda_max(L)) under the standard assumptions of
% connected undirected graphs with nonnegative weights.
info_neighbor_ring = neighbor_only_delay_info(A_ring);
fprintf('Neighbor-only delay model: tau_crit = Inf under connected undirected assumptions.\n');
fprintf('Reason: the model is delay-independent stable in the disagreement subspace.\n\n');

plot_graph_custom(A_ring, [], 'Ring communication graph');
save_current_figure(fullfile(FIG_DIR, 'fig_01_ring_graph.png'));

%% Experiment 1 - 1D delayed consensus
fprintf('========== EXPERIMENT 1: 1D DELAYED CONSENSUS ==========\n');

tau = 0.5 * info_ring.tau_crit;

[time_1d, X_1d] = simulate_delayed_consensus_1d(L_ring, x0, tau, 25.0, 0.01, 1.0);
summary_1d = summarize_consensus_1d(time_1d, X_1d, 1e-2);

fprintf('Delay tau: %.12g\n', tau);
fprintf('Initial average: %.12g\n', mean(x0));
disp('Final states:');
disp(X_1d(end, :)');
disp(summary_1d);

plot_1d_consensus(time_1d, X_1d, '1D delayed consensus on a ring graph');
save_current_figure(fullfile(FIG_DIR, 'fig_02_1d_consensus.png'));

d_1d = disagreement_norm_1d(X_1d);
plot_disagreement_curves(time_1d, d_1d, {'1D consensus'}, '1D disagreement norm');
save_current_figure(fullfile(FIG_DIR, 'fig_02b_1d_disagreement.png'));

%% Experiment 2 - 2D delayed consensus
fprintf('\n========== EXPERIMENT 2: 2D DELAYED CONSENSUS ==========\n');

[time_2d, P_2d] = simulate_delayed_consensus_2d(L_ring, P0, tau, 25.0, 0.01, 1.0);
summary_2d = summarize_consensus_2d(time_2d, P_2d, 1e-2);
centroid = mean(P0, 1);

fprintf('Initial centroid: [%.6f, %.6f]\n', centroid(1), centroid(2));
disp('Final positions:');
disp(squeeze(P_2d(end, :, :)));
disp(summary_2d);

plot_2d_trajectories(P_2d, '2D delayed consensus: rendezvous trajectories', centroid);
save_current_figure(fullfile(FIG_DIR, 'fig_03_2d_rendezvous.png'));

%% Experiment 3 - Delay comparison
fprintf('\n========== EXPERIMENT 3: DELAY COMPARISON ==========\n');

delay_multipliers = [0.0, 0.25, 0.75, 0.95, 1.05];
delay_rows = {};
delay_curves = zeros(length(0:0.01:35.0), length(delay_multipliers));
delay_labels = cell(1, length(delay_multipliers));

for idx = 1:length(delay_multipliers)
    mult = delay_multipliers(idx);
    tau_test = mult * info_ring.tau_crit;

    [time_d, X_d] = simulate_delayed_consensus_1d(L_ring, x0, tau_test, 35.0, 0.01, 1.0);
    d = disagreement_norm_1d(X_d);
    summary = summarize_consensus_1d(time_d, X_d, 1e-2);

    delay_curves(:, idx) = d(:);
    delay_labels{idx} = sprintf('%.2f taucrit', mult);

    delay_rows(end+1, :) = {mult, tau_test, info_ring.tau_crit, tau_test/info_ring.tau_crit, ...
        summary.final_disagreement, summary.max_disagreement, summary.convergence_time};

    fprintf('tau/taucrit = %.2f | tau = %.4f | final disagreement = %.4e | tc = %.4f\n', ...
        mult, tau_test, summary.final_disagreement, summary.convergence_time);
end

write_cell_csv(fullfile(TABLE_DIR, 'delay_comparison.csv'), ...
    {'tau_multiplier','tau','tau_crit','tau_over_taucrit','final_disagreement','max_disagreement','convergence_time'}, ...
    delay_rows);

plot_disagreement_curves(time_d, delay_curves, delay_labels, 'Effect of communication delay on consensus convergence');
save_current_figure(fullfile(FIG_DIR, 'fig_04_delay_sweep.png'));

%% Experiment 4 - Topology comparison
fprintf('\n========== EXPERIMENT 4: TOPOLOGY COMPARISON ==========\n');

graph_types = {'path', 'ring', 'star', 'complete'};
same_tau = 0.08;
topology_rows = {};
topology_curves = zeros(length(0:0.01:25.0), length(graph_types));

for idx = 1:length(graph_types)
    g = graph_types{idx};

    A_g = adjacency_matrix(g, N);
    L_g = laplacian_matrix(A_g);
    info_g = graph_spectral_info(L_g);

    [time_g, X_g] = simulate_delayed_consensus_1d(L_g, x0, same_tau, 25.0, 0.01, 1.0);
    d = disagreement_norm_1d(X_g);
    summary = summarize_consensus_1d(time_g, X_g, 1e-2);

    topology_curves(:, idx) = d(:);

    topology_rows(end+1, :) = {g, info_g.lambda_2, info_g.lambda_max, info_g.tau_crit, same_tau, ...
        summary.final_disagreement, summary.convergence_time};

    fprintf('%-8s | lambda_2 = %.4f | lambda_max = %.4f | taucrit = %.4f | tc = %.4f\n', ...
        g, info_g.lambda_2, info_g.lambda_max, info_g.tau_crit, summary.convergence_time);
end

write_cell_csv(fullfile(TABLE_DIR, 'topology_comparison.csv'), ...
    {'graph','lambda_2','lambda_max','tau_crit','tau_used','final_disagreement','convergence_time'}, ...
    topology_rows);

plot_disagreement_curves(time_g, topology_curves, graph_types, 'Effect of graph topology on convergence');
save_current_figure(fullfile(FIG_DIR, 'fig_05_topology_comparison.png'));

for idx = 1:length(graph_types)
    g = graph_types{idx};
    plot_graph_custom(adjacency_matrix(g, N), [], [g ' graph']);
    save_current_figure(fullfile(FIG_DIR, ['graph_' g '.png']));
end

%% Experiment 5 - Random connected graphs
fprintf('\n========== EXPERIMENT 5: RANDOM CONNECTED GRAPHS ==========\n');

num_trials = 40;
p_edge = 0.45;
random_rows = {};

for trial = 1:num_trials
    A_rand = random_connected_graph(N, p_edge, 5000, 100 + trial);
    L_rand = laplacian_matrix(A_rand);
    info_rand = graph_spectral_info(L_rand);

    tau_rand = 0.5 * info_rand.tau_crit;

    [time_rand, X_rand] = simulate_delayed_consensus_1d(L_rand, x0, tau_rand, 25.0, 0.01, 1.0);
    summary = summarize_consensus_1d(time_rand, X_rand, 1e-2);

    random_rows(end+1, :) = {trial, p_edge, sum(A_rand(:))/2, info_rand.lambda_2, ...
        info_rand.lambda_max, info_rand.tau_crit, tau_rand, 0.5, ...
        summary.final_disagreement, summary.convergence_time};
end

write_cell_csv(fullfile(TABLE_DIR, 'random_connected_graphs.csv'), ...
    {'trial','edge_probability','num_edges','lambda_2','lambda_max','tau_crit','tau_used','tau_over_taucrit','final_disagreement','convergence_time'}, ...
    random_rows);

random_data = cell2mat(random_rows(:, 3:end));
lambda2_vals = random_data(:, 2);
lmax_vals = random_data(:, 3);
taucrit_vals = random_data(:, 4);
tc_vals = random_data(:, 8);

figure;
scatter(lambda2_vals, tc_vals, 45, 'filled');
grid on;
xlabel('\lambda_2(L)');
ylabel('Convergence time [s]');
title('Random connected graphs: algebraic connectivity vs convergence time');
save_current_figure(fullfile(FIG_DIR, 'fig_06_random_lambda2_convergence.png'));

figure;
scatter(lmax_vals, taucrit_vals, 45, 'filled');
grid on;
xlabel('\lambda_{max}(L)');
ylabel('\tau_{crit}');
title('Random connected graphs: largest eigenvalue vs delay bound');
save_current_figure(fullfile(FIG_DIR, 'fig_07_random_lmax_taucrit.png'));

fprintf('Random graph experiment completed with %d trials.\n', num_trials);

%% Experiment 6 - Random geometric graphs
fprintf('\n========== EXPERIMENT 6: RANDOM GEOMETRIC GRAPHS ==========\n');

radii = [4.0, 5.0, 6.0];
trials_per_radius = 15;
geometric_rows = {};

for r_idx = 1:length(radii)
    radius = radii(r_idx);

    for trial = 1:trials_per_radius
        [A_geo, pos_geo] = random_geometric_graph(N, radius, 8.0, 5000, 500 + 100*r_idx + trial);
        L_geo = laplacian_matrix(A_geo);
        info_geo = graph_spectral_info(L_geo);

        tau_geo = 0.5 * info_geo.tau_crit;

        [time_geo, X_geo] = simulate_delayed_consensus_1d(L_geo, x0, tau_geo, 25.0, 0.01, 1.0);
        summary = summarize_consensus_1d(time_geo, X_geo, 1e-2);

        geometric_rows(end+1, :) = {radius, trial, sum(A_geo(:))/2, info_geo.lambda_2, ...
            info_geo.lambda_max, info_geo.tau_crit, tau_geo, ...
            summary.final_disagreement, summary.convergence_time};
    end
end

write_cell_csv(fullfile(TABLE_DIR, 'random_geometric_graphs.csv'), ...
    {'radius','trial','num_edges','lambda_2','lambda_max','tau_crit','tau_used','final_disagreement','convergence_time'}, ...
    geometric_rows);

geo_numeric = cell2mat(geometric_rows);
figure;
hold on;
for r_idx = 1:length(radii)
    radius = radii(r_idx);
    mask = abs(geo_numeric(:,1) - radius) < 1e-9;
    scatter(geo_numeric(mask,3), geo_numeric(mask,9), 45, 'filled');
end
grid on;
xlabel('Number of edges');
ylabel('Convergence time [s]');
title('Random geometric graphs: communication density vs convergence');
legend({'R=4','R=5','R=6'});
save_current_figure(fullfile(FIG_DIR, 'fig_08_geometric_edges_convergence.png'));

[A_geo_example, pos_geo_example] = random_geometric_graph(N, 5.0, 8.0, 5000, 999);
plot_graph_custom(A_geo_example, pos_geo_example, 'Example random geometric graph');
save_current_figure(fullfile(FIG_DIR, 'fig_08b_example_geometric_graph.png'));

fprintf('Random geometric graph experiment completed.\n');

%% Experiment 7 - Disconnected graph
fprintf('\n========== EXPERIMENT 7: DISCONNECTED GRAPH ==========\n');

A_disc = adjacency_matrix('disconnected', N);
L_disc = laplacian_matrix(A_disc);
info_disc = graph_spectral_info(L_disc);

fprintf('Disconnected graph eigenvalues:\n');
disp(info_disc.eigvals');
fprintf('Number of zero eigenvalues: %d\n', info_disc.zero_count);
fprintf('Connected: %d\n', info_disc.is_connected);

[time_disc, P_disc] = simulate_delayed_consensus_2d(L_disc, P0, 0.1, 25.0, 0.01, 1.0);
summary_disc = summarize_consensus_2d(time_disc, P_disc, 1e-2);

disp('Final positions:');
disp(squeeze(P_disc(end, :, :)));
fprintf('Final global disagreement: %.6f\n', summary_disc.final_disagreement);

plot_graph_custom(A_disc, [], 'Disconnected communication graph');
save_current_figure(fullfile(FIG_DIR, 'fig_09_disconnected_graph.png'));

plot_2d_trajectories(P_disc, 'Disconnected graph: local rendezvous but no global rendezvous', []);
save_current_figure(fullfile(FIG_DIR, 'fig_09_disconnected_trajectories.png'));

%% Experiment 8 - Integration-step sensitivity
fprintf('\n========== EXPERIMENT 8: INTEGRATION-STEP SENSITIVITY ==========\n');

dt_values = [0.02, 0.01, 0.005];
dt_rows = {};
dt_labels = cell(1, length(dt_values));
dt_curves_cell = cell(1, length(dt_values));
time_cell = cell(1, length(dt_values));

tau_dt = 0.5 * info_ring.tau_crit;

for idx = 1:length(dt_values)
    dt_test = dt_values(idx);

    [time_dt, X_dt] = simulate_delayed_consensus_1d(L_ring, x0, tau_dt, 25.0, dt_test, 1.0);
    summary = summarize_consensus_1d(time_dt, X_dt, 1e-2);
    d = disagreement_norm_1d(X_dt);

    dt_rows(end+1, :) = {dt_test, tau_dt, summary.final_disagreement, ...
        summary.convergence_time, mean(X_dt(end, :))};

    dt_labels{idx} = sprintf('dt=%.3f', dt_test);
    dt_curves_cell{idx} = d;
    time_cell{idx} = time_dt;
end

write_cell_csv(fullfile(TABLE_DIR, 'integration_step_sensitivity.csv'), ...
    {'dt','tau','final_disagreement','convergence_time','final_average'}, ...
    dt_rows);

plot_disagreement_curves_variable_time(time_cell, dt_curves_cell, dt_labels, 'Integration-step sensitivity');
save_current_figure(fullfile(FIG_DIR, 'fig_10_dt_sensitivity.png'));

%% Experiment 9 - Unicycle rendezvous
fprintf('\n========== EXPERIMENT 9: UNICYCLE RENDEZVOUS ==========\n');

tau_uni = 0.3 * info_ring.tau_crit;

[time_uni, states_uni, commands_uni] = simulate_unicycle_rendezvous(...
    L_ring, initial_unicycle_states, tau_uni, 35.0, 0.01, ...
    1.0, 1.0, 3.0, true, 1.5, 4.0, true);

P_uni = states_uni(:, :, 1:2);
summary_uni = summarize_consensus_2d(time_uni, P_uni, 1e-2);

fprintf('Delay tau: %.6f\n', tau_uni);
fprintf('Initial centroid: [%.6f, %.6f]\n', mean(initial_unicycle_states(:,1)), mean(initial_unicycle_states(:,2)));
fprintf('Final rendezvous point approximately: [%.6f, %.6f]\n', ...
    mean(P_uni(end,:,1)), mean(P_uni(end,:,2)));
fprintf('Final disagreement: %.6e\n', summary_uni.final_disagreement);
fprintf('Convergence time: %.6f\n', summary_uni.convergence_time);

plot_unicycle_trajectories(states_uni, 'Unicycle rendezvous with delayed communication', true, 450);
save_current_figure(fullfile(FIG_DIR, 'fig_11_unicycle_rendezvous.png'));

d_uni = disagreement_norm_2d(P_uni);
plot_disagreement_curves(time_uni, d_uni, {'unicycle positions'}, 'Unicycle rendezvous disagreement');
save_current_figure(fullfile(FIG_DIR, 'fig_11b_unicycle_disagreement.png'));

plot_unicycle_commands(time_uni, commands_uni);
save_current_figure(fullfile(FIG_DIR, 'fig_11c_unicycle_commands.png'));

%% Experiment 10 - Saturated vs unsaturated unicycle
fprintf('\n========== EXPERIMENT 10: SATURATED VS UNSATURATED UNICYCLE ==========\n');

unicycle_rows = {};
unicycle_case_names = {'unsaturated', 'saturated'};
unicycle_curves = zeros(length(time_uni), 2);

for idx = 1:2
    if idx == 1
        use_sat = false;
        v_max = 1e9;
        omega_max = 1e9;
    else
        use_sat = true;
        v_max = 1.5;
        omega_max = 4.0;
    end

    [time_case, states_case, commands_case] = simulate_unicycle_rendezvous(...
        L_ring, initial_unicycle_states, tau_uni, 35.0, 0.01, ...
        1.0, 1.0, 3.0, use_sat, v_max, omega_max, true);

    P_case = states_case(:, :, 1:2);
    summary = summarize_consensus_2d(time_case, P_case, 1e-2);
    d = disagreement_norm_2d(P_case);

    unicycle_curves(:, idx) = d(:);

    unicycle_rows(end+1, :) = {unicycle_case_names{idx}, summary.final_disagreement, ...
        summary.convergence_time, mean(P_case(end,:,1)), mean(P_case(end,:,2)), ...
        max(max(abs(commands_case(:,:,1)))), max(max(abs(commands_case(:,:,2))))};
end

write_cell_csv(fullfile(TABLE_DIR, 'unicycle_saturated_vs_unsaturated.csv'), ...
    {'case','final_disagreement','convergence_time','final_x','final_y','max_abs_v','max_abs_omega'}, ...
    unicycle_rows);

plot_disagreement_curves(time_uni, unicycle_curves, unicycle_case_names, 'Saturated vs unsaturated unicycle rendezvous');
save_current_figure(fullfile(FIG_DIR, 'fig_11d_unicycle_saturation_comparison.png'));

%% Experiment 11 - Weighted a_ij
fprintf('\n========== EXPERIMENT 11: WEIGHTED GRAPHS ==========\n');

A_binary_ring = A_ring;
L_binary_ring = L_ring;
info_binary_ring = info_ring;

ring_weights = [0.4, 1.2, 0.7, 1.8, 1.0, 1.5];
A_weighted_ring = weighted_ring_graph(N, ring_weights);
L_weighted_ring = laplacian_matrix(A_weighted_ring);
info_weighted_ring = graph_spectral_info(L_weighted_ring);

print_graph_info(A_binary_ring, 'Binary ring, a_ij = 1');
print_graph_info(A_weighted_ring, 'Weighted ring, nonuniform a_ij');

plot_graph_custom(A_weighted_ring, [], 'Weighted ring graph: same topology, different a_{ij}');
save_current_figure(fullfile(FIG_DIR, 'fig_12_weighted_ring_graph.png'));

weighted_rows = {};
weighted_curves = zeros(length(0:0.01:25.0), 2);
weighted_labels = {'binary ring', 'weighted ring'};

for idx = 1:2
    if idx == 1
        L_w = L_binary_ring;
        info_w = info_binary_ring;
        name = 'binary_ring';
    else
        L_w = L_weighted_ring;
        info_w = info_weighted_ring;
        name = 'weighted_ring';
    end

    tau_used = 0.5 * info_w.tau_crit;
    [time_w, X_w] = simulate_delayed_consensus_1d(L_w, x0, tau_used, 25.0, 0.01, 1.0);
    d = disagreement_norm_1d(X_w);
    summary = summarize_consensus_1d(time_w, X_w, 1e-2);

    weighted_curves(:, idx) = d(:);

    weighted_rows(end+1, :) = {name, info_w.lambda_2, info_w.lambda_max, info_w.tau_crit, ...
        tau_used, summary.final_disagreement, summary.convergence_time, mean(X_w(end,:))};
end

write_cell_csv(fullfile(TABLE_DIR, 'weighted_vs_binary_ring.csv'), ...
    {'case','lambda_2','lambda_max','tau_crit','tau_used','final_disagreement','convergence_time','final_average'}, ...
    weighted_rows);

plot_disagreement_curves(time_w, weighted_curves, weighted_labels, 'Binary vs weighted ring: full-state delayed consensus');
save_current_figure(fullfile(FIG_DIR, 'fig_13_weighted_vs_binary.png'));

%% Experiment 12 - Weighted graph sweep
fprintf('\n========== EXPERIMENT 12: WEIGHTED GRAPH SWEEP ==========\n');

weighted_sweep_rows = {};
num_weighted_trials = 40;

for trial = 1:num_weighted_trials
    A_w = apply_symmetric_random_weights(A_binary_ring, 0.3, 2.0, 800 + trial);
    L_w = laplacian_matrix(A_w);
    info_w = graph_spectral_info(L_w);
    tau_used = 0.5 * info_w.tau_crit;

    [time_ws, X_ws] = simulate_delayed_consensus_1d(L_w, x0, tau_used, 25.0, 0.01, 1.0);
    summary = summarize_consensus_1d(time_ws, X_ws, 1e-2);

    mean_weight = sum(A_w(:)) / sum(A_binary_ring(:));

    weighted_sweep_rows(end+1, :) = {trial, 0.3, 2.0, mean_weight, info_w.lambda_2, ...
        info_w.lambda_max, info_w.tau_crit, tau_used, summary.final_disagreement, summary.convergence_time};
end

write_cell_csv(fullfile(TABLE_DIR, 'weighted_graph_sweep.csv'), ...
    {'trial','w_min','w_max','mean_weight','lambda_2','lambda_max','tau_crit','tau_used','final_disagreement','convergence_time'}, ...
    weighted_sweep_rows);

ws_data = cell2mat(weighted_sweep_rows);
figure;
scatter(ws_data(:,5), ws_data(:,10), 45, 'filled');
grid on;
xlabel('\lambda_2(L)');
ylabel('Convergence time [s]');
title('Weighted graphs: lambda_2 vs convergence time');
save_current_figure(fullfile(FIG_DIR, 'fig_13b_weighted_lambda2_convergence.png'));

figure;
scatter(ws_data(:,6), ws_data(:,7), 45, 'filled');
grid on;
xlabel('\lambda_{max}(L)');
ylabel('\tau_{crit}');
title('Weighted graphs: lambda_max vs delay bound');
save_current_figure(fullfile(FIG_DIR, 'fig_13c_weighted_lmax_taucrit.png'));

%% Experiment 13 - Full-state delay vs neighbor-only delay
fprintf('\n========== EXPERIMENT 13: FULL DELAY VS NEIGHBOR-ONLY DELAY ==========\n');

tau_compare = 0.75 * info_ring.tau_crit;

[time_full, X_full] = simulate_delayed_consensus_1d(L_ring, x0, tau_compare, 35.0, 0.01, 1.0);
[time_neigh, X_neigh] = simulate_neighbor_only_delay_consensus_1d(A_ring, x0, tau_compare, 35.0, 0.01, 1.0);

summary_full = summarize_consensus_1d(time_full, X_full, 1e-2);
summary_neigh = summarize_consensus_1d(time_neigh, X_neigh, 1e-2);

delay_model_rows = {...
    'full_state_delay', tau_compare, summary_full.final_disagreement, summary_full.convergence_time, mean(x0), mean(X_full(end,:)); ...
    'neighbor_only_delay', tau_compare, summary_neigh.final_disagreement, summary_neigh.convergence_time, mean(x0), mean(X_neigh(end,:))};

write_cell_csv(fullfile(TABLE_DIR, 'full_delay_vs_neighbor_only_delay_1d.csv'), ...
    {'model','tau','final_disagreement','convergence_time','initial_average','final_average'}, ...
    delay_model_rows);

plot_disagreement_curves(time_full, [disagreement_norm_1d(X_full), disagreement_norm_1d(X_neigh)], ...
    {'full-state delay', 'neighbor-only delay'}, 'Full-state delay vs neighbor-only delay, 1D');
save_current_figure(fullfile(FIG_DIR, 'fig_14_delay_model_1d.png'));

%% Experiment 14 - Delay sweep for alternative model
fprintf('\n========== EXPERIMENT 14: DELAY MODEL SWEEP ==========\n');

delay_model_sweep_rows = {};
curves_neigh = zeros(length(0:0.01:35.0), length(delay_multipliers));
curves_full = zeros(length(0:0.01:35.0), length(delay_multipliers));

for idx = 1:length(delay_multipliers)
    mult = delay_multipliers(idx);
    tau_test = mult * info_ring.tau_crit;

    [time_f, X_f] = simulate_delayed_consensus_1d(L_ring, x0, tau_test, 35.0, 0.01, 1.0);
    [time_n, X_n] = simulate_neighbor_only_delay_consensus_1d(A_ring, x0, tau_test, 35.0, 0.01, 1.0);

    summary_f = summarize_consensus_1d(time_f, X_f, 1e-2);
    summary_n = summarize_consensus_1d(time_n, X_n, 1e-2);

    curves_full(:, idx) = disagreement_norm_1d(X_f);
    curves_neigh(:, idx) = disagreement_norm_1d(X_n);

    delay_model_sweep_rows(end+1, :) = {mult, tau_test, 'full_state_delay', ...
        summary_f.final_disagreement, summary_f.max_disagreement, summary_f.convergence_time, mean(X_f(end,:))};
    delay_model_sweep_rows(end+1, :) = {mult, tau_test, 'neighbor_only_delay', ...
        summary_n.final_disagreement, summary_n.max_disagreement, summary_n.convergence_time, mean(X_n(end,:))};
end

write_cell_csv(fullfile(TABLE_DIR, 'delay_model_sweep_full_vs_neighbor_only.csv'), ...
    {'tau_multiplier','tau','model','final_disagreement','max_disagreement','convergence_time','final_average'}, ...
    delay_model_sweep_rows);

plot_disagreement_curves(time_f, curves_full, delay_labels, 'Delay sweep: full-state delayed model');
save_current_figure(fullfile(FIG_DIR, 'fig_15a_full_delay_sweep.png'));

plot_disagreement_curves(time_n, curves_neigh, delay_labels, 'Delay sweep: neighbor-only delayed model');
save_current_figure(fullfile(FIG_DIR, 'fig_15_neighbor_delay_sweep.png'));

%% Experiment 15 - 2D comparison of delay models
fprintf('\n========== EXPERIMENT 15: 2D DELAY MODEL COMPARISON ==========\n');

tau_compare_2d = 0.75 * info_ring.tau_crit;

[time_full_2d, P_full_2d] = simulate_delayed_consensus_2d(L_ring, P0, tau_compare_2d, 35.0, 0.01, 1.0);
[time_neigh_2d, P_neigh_2d] = simulate_neighbor_only_delay_consensus_2d(A_ring, P0, tau_compare_2d, 35.0, 0.01, 1.0);

summary_full_2d = summarize_consensus_2d(time_full_2d, P_full_2d, 1e-2);
summary_neigh_2d = summarize_consensus_2d(time_neigh_2d, P_neigh_2d, 1e-2);

delay_model_2d_rows = {...
    'full_state_delay', tau_compare_2d, summary_full_2d.final_disagreement, summary_full_2d.convergence_time, ...
    mean(P0(:,1)), mean(P0(:,2)), mean(P_full_2d(end,:,1)), mean(P_full_2d(end,:,2)); ...
    'neighbor_only_delay', tau_compare_2d, summary_neigh_2d.final_disagreement, summary_neigh_2d.convergence_time, ...
    mean(P0(:,1)), mean(P0(:,2)), mean(P_neigh_2d(end,:,1)), mean(P_neigh_2d(end,:,2))};

write_cell_csv(fullfile(TABLE_DIR, 'full_delay_vs_neighbor_only_delay_2d.csv'), ...
    {'model','tau','final_disagreement','convergence_time','initial_centroid_x','initial_centroid_y','final_centroid_x','final_centroid_y'}, ...
    delay_model_2d_rows);

plot_2d_trajectories(P_full_2d, '2D rendezvous: full-state delayed model', centroid);
save_current_figure(fullfile(FIG_DIR, 'fig_15b_2d_full_delay.png'));

plot_2d_trajectories(P_neigh_2d, '2D rendezvous: neighbor-only delayed model', centroid);
save_current_figure(fullfile(FIG_DIR, 'fig_15c_2d_neighbor_delay.png'));

plot_disagreement_curves(time_full_2d, [disagreement_norm_2d(P_full_2d), disagreement_norm_2d(P_neigh_2d)], ...
    {'full-state delay', 'neighbor-only delay'}, '2D delay model comparison');
save_current_figure(fullfile(FIG_DIR, 'fig_15d_2d_delay_model_disagreement.png'));

%% Experiment 16 - Unicycle comparison with neighbor-only delay
fprintf('\n========== EXPERIMENT 16: UNICYCLE DELAY MODEL COMPARISON ==========\n');

tau_uni_compare = 0.3 * info_ring.tau_crit;

[time_uni_full, states_uni_full, commands_uni_full] = simulate_unicycle_rendezvous(...
    L_ring, initial_unicycle_states, tau_uni_compare, 35.0, 0.01, ...
    1.0, 1.0, 3.0, true, 1.5, 4.0, true);

[time_uni_neigh, states_uni_neigh, commands_uni_neigh] = simulate_unicycle_neighbor_only_delay(...
    A_ring, initial_unicycle_states, tau_uni_compare, 35.0, 0.01, ...
    1.0, 1.0, 3.0, true, 1.5, 4.0, true);

P_uni_full = states_uni_full(:, :, 1:2);
P_uni_neigh = states_uni_neigh(:, :, 1:2);

summary_uni_full = summarize_consensus_2d(time_uni_full, P_uni_full, 1e-2);
summary_uni_neigh = summarize_consensus_2d(time_uni_neigh, P_uni_neigh, 1e-2);

unicycle_delay_model_rows = {...
    'full_state_delay_unicycle', tau_uni_compare, summary_uni_full.final_disagreement, summary_uni_full.convergence_time, ...
    mean(P_uni_full(end,:,1)), mean(P_uni_full(end,:,2)), max(max(abs(commands_uni_full(:,:,1)))), max(max(abs(commands_uni_full(:,:,2)))); ...
    'neighbor_only_delay_unicycle', tau_uni_compare, summary_uni_neigh.final_disagreement, summary_uni_neigh.convergence_time, ...
    mean(P_uni_neigh(end,:,1)), mean(P_uni_neigh(end,:,2)), max(max(abs(commands_uni_neigh(:,:,1)))), max(max(abs(commands_uni_neigh(:,:,2))))};

write_cell_csv(fullfile(TABLE_DIR, 'unicycle_full_delay_vs_neighbor_only_delay.csv'), ...
    {'model','tau','final_disagreement','convergence_time','final_x','final_y','max_abs_v','max_abs_omega'}, ...
    unicycle_delay_model_rows);

plot_unicycle_trajectories(states_uni_full, 'Unicycle: full-state delayed consensus', true, 450);
save_current_figure(fullfile(FIG_DIR, 'fig_16a_unicycle_full_delay.png'));

plot_unicycle_trajectories(states_uni_neigh, 'Unicycle: neighbor-only delayed consensus', true, 450);
save_current_figure(fullfile(FIG_DIR, 'fig_16b_unicycle_neighbor_delay.png'));

plot_disagreement_curves(time_uni_full, [disagreement_norm_2d(P_uni_full), disagreement_norm_2d(P_uni_neigh)], ...
    {'full-state delay unicycle', 'neighbor-only delay unicycle'}, 'Unicycle delay model comparison');
save_current_figure(fullfile(FIG_DIR, 'fig_16_unicycle_delay_model.png'));

%% Videos / animation
if MAKE_VIDEOS
    fprintf('\n========== VIDEO GENERATION ==========\n');

    [time_video_stable, P_video_stable] = simulate_delayed_consensus_2d(...
        L_ring, P0, 0.5 * info_ring.tau_crit, 16.0, 0.02, 1.0);

    save_2d_animation(P_video_stable, time_video_stable, ...
        fullfile(VIDEO_DIR, 'video_1_stable_2d_consensus.mp4'), ...
        'Stable delayed 2D consensus', [-4, 5], [-4, 4], 90, true);

    [time_video_unstable, P_video_unstable] = simulate_delayed_consensus_2d(...
        L_ring, P0, 1.05 * info_ring.tau_crit, 10.0, 0.01, 1.0);

    save_2d_animation(P_video_unstable, time_video_unstable, ...
        fullfile(VIDEO_DIR, 'video_2_unstable_2d_consensus.mp4'), ...
        'Unstable delayed 2D consensus', [-10, 10], [-10, 10], 90, true);

    [time_video_uni, states_video_uni, ~] = simulate_unicycle_rendezvous(...
        L_ring, initial_unicycle_states, 0.3 * info_ring.tau_crit, 22.0, 0.02, ...
        1.0, 1.0, 3.0, true, 1.5, 4.0, true);

    save_unicycle_animation(states_video_uni, time_video_uni, ...
        fullfile(VIDEO_DIR, 'video_3_unicycle_rendezvous.mp4'), ...
        'Unicycle rendezvous with delayed communication', [-4, 5], [-4, 4], 100);

    [time_video_disc, P_video_disc] = simulate_delayed_consensus_2d(...
        L_disc, P0, 0.1, 22.0, 0.02, 1.0);

    save_2d_animation(P_video_disc, time_video_disc, ...
        fullfile(VIDEO_DIR, 'video_4_disconnected_graph.mp4'), ...
        'Disconnected graph: no global rendezvous', [-4, 5], [-4, 4], 90, false);

    save_delay_model_comparison_animation(P_full_2d, P_neigh_2d, time_full_2d, ...
        fullfile(VIDEO_DIR, 'video_5_full_delay_vs_neighbor_only_delay.mp4'), ...
        'Full-state delay', 'Neighbor-only delay', [-4, 5], [-4, 4], 100);
end

%% Final summary
fprintf('\n========== FINAL PROJECT SUMMARY ==========\n');
fprintf('1. 1D and 2D delayed consensus converge below the theoretical bound.\n');
fprintf('2. Delay sweep confirms slower/oscillatory convergence near tau_crit and instability above tau_crit.\n');
fprintf('3. Topology affects convergence through lambda_2 and delay margin through lambda_max.\n');
fprintf('4. Random and geometric graphs confirm the spectral interpretation.\n');
fprintf('5. Disconnected graphs lead to local, not global, rendezvous.\n');
fprintf('6. Weighted a_ij modify the Laplacian spectrum and therefore the dynamics.\n');
fprintf('7. Neighbor-only delay has dynamics xdot = -D x(t) + A x(t-tau), different from -L x(t-tau).\n');
fprintf('8. Unicycle robots reach rendezvous after converting consensus velocity to v and omega.\n');
fprintf('\nMain theoretical relation for full-state delay:\n');
fprintf('tau_crit = pi / (2 * lambda_max(L))\n');
fprintf('\nGenerated outputs are in: %s\n', OUTPUT_DIR);
fprintf('\nIf Octave saved *_frames folders instead of mp4 videos, convert them with:\n');
fprintf('  convert_all_frames_to_mp4(''p1_outputs_matlab/videos'', 15)\n');
fprintf('or from shell:\n');
fprintf('  ./convert_frames_to_mp4.sh p1_outputs_matlab/videos 15\n');
