# P1 — Rendezvous of Unicycles with Communication Delays

**Course:** Control of Multi-Robot Systems — Project 2026
**Author:** Samuele Civale
**Project:** P1 — Rendezvous of Unicycles with Communication Delays
**Final version:** v3 — MATLAB/Octave implementation with realistic extensions

---

## 1. Project overview

This repository contains a complete simulation framework for studying the **rendezvous problem** for a group of mobile robots communicating through an undirected graph in the presence of **communication delays**.

The project starts from the delayed consensus model provided in the project statement and develops it into a full experimental pipeline. The work includes:

* graph-theoretic modeling of multi-robot communication;
* derivation and numerical verification of the delay stability threshold;
* comparison between several communication topologies;
* analysis of weighted, random and geometric graphs;
* comparison between **full-state delay** and **neighbor-only delay** models;
* extension from ideal single-integrator agents to **unicycle robots**;
* realistic v3 extensions including edge-specific delays, time-varying delays, switching graphs, collision avoidance, obstacle avoidance, packet loss, dynamic unicycle behavior and connectivity maintenance.

The repository is designed to be executable in **MATLAB** and mostly compatible with **GNU Octave**.

---

## 2. Main idea

The basic rendezvous problem asks a group of agents to converge to a common point using only local information from neighboring agents.

For an undirected graph (G = (V,E)), the standard continuous-time consensus law without delay is:

[
\dot{x}*i(t) = -\sum*{j \in \mathcal{N}*i} a*{ij}(x_i(t)-x_j(t)).
]

In matrix form:

[
\dot{x}(t) = -Lx(t),
]

where (L = D-A) is the graph Laplacian.

The delayed full-state model studied in the project is:

[
\dot{x}(t) = -Lx(t-\tau).
]

For a connected undirected graph, the disagreement modes are stable if:

[
\tau < \tau_{crit} = \frac{\pi}{2\lambda_{\max}(L)}.
]

This threshold is one of the central theoretical results of the project.

---

## 3. Full-state delay vs neighbor-only delay

The project also studies a more realistic communication model where each robot knows its own current state but receives delayed information from its neighbors:

[
\dot{x}(t) = -Dx(t) + Ax(t-\tau).
]

This is called the **neighbor-only delay model**.

The key conceptual difference is:

| Model               | Equation                       | Interpretation                       |
| ------------------- | ------------------------------ | ------------------------------------ |
| Full-state delay    | (\dot{x}(t)=-Lx(t-\tau))       | the whole consensus term is delayed  |
| Neighbor-only delay | (\dot{x}(t)=-Dx(t)+Ax(t-\tau)) | only neighbor information is delayed |

The full-state model has a finite critical delay:

[
\tau_{crit} = \frac{\pi}{2\lambda_{\max}(L)}.
]

The neighbor-only delay model, under the assumptions of connected undirected graph and nonnegative weights, does not show the same finite critical delay for the disagreement modes. This is because the self-feedback term (-Dx(t)) remains instantaneous.

---

## 4. Repository structure

The repository is organized as follows:

```text
P1_Rendezvous_MATLAB_v3_all_extensions/
│
├── main.m
├── README.md
├── Project_overview.pdf
│
├── adjacency_matrix.m
├── laplacian_matrix.m
├── graph_spectral_info.m
├── is_connected_graph.m
├── apply_symmetric_random_weights.m
│
├── disagreement_norm_1d.m
├── disagreement_norm_2d.m
├── convergence_time_metric.m
├── ensure_dir.m
│
├── simulate_delayed_consensus_1d.m
├── simulate_delayed_consensus_2d.m
├── simulate_neighbor_only_delay_1d.m
├── simulate_neighbor_only_delay_2d.m
├── simulate_unicycle_rendezvous.m
├── simulate_unicycle_neighbor_only_delay.m
│
├── simulate_edge_delayed_consensus_2d.m
├── simulate_time_varying_delay_consensus_2d.m
├── simulate_switching_geometric_consensus_2d.m
├── simulate_packet_loss_consensus_2d.m
├── simulate_unicycle_with_avoidance.m
├── simulate_dynamic_unicycle_rendezvous.m
├── simulate_switching_with_connectivity_maintenance.m
│
├── adjacency_from_positions.m
├── pairwise_min_distance.m
├── min_obstacle_clearance.m
├── split_disturbance_connectivity.m
├── clip_vector_norm.m
│
├── plot_*.m
├── save_*.m
├── convert_all_frames_to_mp4.m
├── convert_frames_to_mp4.sh
│
└── p1_outputs_matlab/
    ├── figures/
    ├── tables/
    └── videos/
```

The exact number of helper files may vary slightly depending on the exported version, but the project is built around a single entry point:

```matlab
main.m
```

---

## 5. Requirements

### MATLAB

Recommended:

* MATLAB R2020b or newer;
* base MATLAB functions;
* no mandatory toolbox is required for the main simulations.

For video generation, MATLAB may use `VideoWriter` when available. If `VideoWriter` is not available or fails, the code saves video frames as `.png` images.

### GNU Octave

The code is written to be as close as possible to MATLAB plain syntax. Most numerical simulations should work in Octave.

For figures and video export, behavior may depend on the local Octave installation.

### External tools for video conversion

If MATLAB/Octave saves frame folders instead of direct `.mp4` videos, install `ffmpeg`.

On macOS with Homebrew:

```bash
brew install ffmpeg
```

If `ffmpeg` is already installed but broken after an update:

```bash
brew update
brew reinstall x265
brew reinstall ffmpeg
```

---

## 6. How to run the project

Open MATLAB or Octave in the project root folder:

```text
P1_Rendezvous_MATLAB_v3_all_extensions/
```

Then run:

```matlab
main
```

The script automatically executes all experiments and saves outputs inside:

```text
p1_outputs_matlab/
```

The main output folders are:

```text
p1_outputs_matlab/figures/
p1_outputs_matlab/tables/
p1_outputs_matlab/videos/
```

---

## 7. Fast mode: disable video generation

Video generation can be slow, especially when MATLAB saves hundreds of frames instead of direct `.mp4` files.

Inside `main.m`, use:

```matlab
MAKE_VIDEOS = false;
```

This runs all numerical experiments and saves figures/tables, but skips video generation.

To generate videos later:

```matlab
MAKE_VIDEOS = true;
```

Recommended workflow:

1. run the project once with `MAKE_VIDEOS = false`;
2. check figures and tables;
3. enable videos only at the end;
4. convert saved frames to `.mp4` if necessary.

---

## 8. Experiments included

### Original experiments

The original part of the project includes:

1. **Consensus 1D with delay**
   Verifies convergence below the theoretical threshold.

2. **Consensus 2D with delay**
   Extends the same consensus law to planar positions.

3. **Delay sweep**
   Compares different values of (\tau/\tau_{crit}), showing stable, oscillatory and unstable behavior.

4. **Topology comparison**
   Compares path, ring, star and complete graphs.

5. **Random connected graphs**
   Tests robustness of the spectral interpretation on irregular connected graphs.

6. **Random geometric graphs**
   Models distance-based communication between agents.

7. **Disconnected graph**
   Shows that global connectivity is necessary for global rendezvous.

8. **Integration step sensitivity**
   Checks whether the numerical results depend strongly on the integration step.

9. **Unicycle rendezvous**
   Converts the desired Cartesian consensus velocity into unicycle commands (v_i) and (\omega_i).

10. **Saturated vs unsaturated unicycle**
    Compares ideal commands and bounded commands.

11. **Weighted graphs**
    Studies non-binary communication weights (a_{ij}\neq 1).

12. **Weighted graph sweep**
    Tests different random weighted graphs.

13. **Full-state delay vs neighbor-only delay in 1D**
    Compares the two delay models.

14. **Full-state delay vs neighbor-only delay in 2D**
    Extends the delay model comparison to planar rendezvous.

15. **Unicycle with neighbor-only delay**
    Tests the neighbor-only delay model with unicycle kinematics.

---

## 9. V3 realistic extensions

The final version adds a realistic second part with additional experiments.

### 9.1 Edge-specific delays

Instead of using a single uniform delay (\tau), each communication edge has its own delay:

[
\tau_{ij}.
]

This models a more realistic network where communication links may have different latencies.

Output files:

```text
edge_specific_delays.csv
video_6_v3_edge_specific_delays_frames/
```

---

### 9.2 Time-varying delay

The delay changes over time:

[
\tau(t)=0.45\tau_{crit}+0.25\tau_{crit}\sin\left(\frac{2\pi t}{8}\right).
]

This tests whether convergence is preserved when delay is not constant.

Output files:

```text
video_7_v3_time_varying_delay_frames/
```

---

### 9.3 Switching geometric graph

The graph changes during the simulation according to the agents' relative distance:

[
(i,j)\in E(t) \Longleftrightarrow |p_i(t)-p_j(t)|\le R.
]

This is closer to a real multi-robot network, where robots communicate only when they are close enough.

Output files:

```text
video_8_v3_switching_geometric_graph_frames/
```

---

### 9.4 Collision avoidance

A repulsive term is added to avoid agent-agent collisions:

[
u_i = u_i^{consensus} + u_i^{repulsive}.
]

In this experiment, zero disagreement is no longer the only objective. The system should form a compact cluster while avoiding physical overlap.

Output files:

```text
video_9_v3_collision_avoidance_frames/
```

---

### 9.5 Obstacle avoidance

A circular obstacle is introduced in the environment. The control law becomes:

[
u_i = u_i^{consensus} + u_i^{obs}.
]

The goal is to move toward rendezvous while keeping a positive clearance from the obstacle.

Output files:

```text
video_10_v3_obstacle_avoidance_frames/
```

---

### 9.6 Packet loss

Messages are randomly lost with probability (p_{loss}). The effective adjacency changes over time depending on which packets are received.

Tested values include:

```text
p_loss = 0.0
p_loss = 0.1
p_loss = 0.3
p_loss = 0.5
```

Output files:

```text
packet_loss_sweep.csv
video_11_v3_packet_loss_50_percent_frames/
```

---

### 9.7 Dynamic unicycle with acceleration limits

The original unicycle model is kinematic:

[
\dot{x}_i = v_i\cos\theta_i,\quad
\dot{y}_i = v_i\sin\theta_i,\quad
\dot{\theta}_i = \omega_i.
]

The v3 dynamic version adds:

[
\dot{v}_i = a_i,\qquad
\dot{\omega}_i = \alpha_i.
]

This prevents instantaneous changes in velocity and angular velocity.

Output files:

```text
video_12_v3_dynamic_unicycle_frames/
```

---

### 9.8 Connectivity maintenance using (\lambda_2)

The algebraic connectivity (\lambda_2(L(t))) is monitored during the simulation.

Connectivity is preserved if:

[
\lambda_2(L(t)) > 0.
]

The project compares a case without connectivity maintenance and a case with a maintenance term activated when (\lambda_2) becomes too small.

Output files:

```text
connectivity_maintenance_comparison.csv
video_13_v3_connectivity_maintenance_comparison_frames/
```

---

## 10. Output tables

The project saves numerical results as `.csv` files inside:

```text
p1_outputs_matlab/tables/
```

or, depending on the version:

```text
p1_outputs/
```

Typical generated tables include:

```text
delay_comparison.csv
topology_comparison.csv
random_connected_graphs.csv
random_geometric_graphs.csv
integration_step_sensitivity.csv
unicycle_saturated_vs_unsaturated.csv
weighted_vs_binary_ring.csv
weighted_graph_sweep.csv
full_delay_vs_neighbor_only_delay_1d.csv
full_delay_vs_neighbor_only_delay_2d.csv
delay_model_sweep_full_vs_neighbor_only.csv
unicycle_full_delay_vs_neighbor_only_delay.csv
edge_specific_delays.csv
packet_loss_sweep.csv
connectivity_maintenance_comparison.csv
```

---

## 11. Output figures

Figures are saved in:

```text
p1_outputs_matlab/figures/
```

They include:

* communication graph plots;
* 1D consensus trajectories;
* 2D rendezvous trajectories;
* delay sweep plots;
* topology comparisons;
* random graph spectral plots;
* geometric graph results;
* weighted graph comparisons;
* full-state vs neighbor-only delay comparisons;
* v3 extension plots.

All time plots use:

```text
Time [s]
```

as the x-axis label.

---

## 12. Video generation

The project generates video animations for the main simulations.

The expected video set is:

```text
video_1_stable_2d_consensus.mp4
video_2_unstable_2d_consensus.mp4
video_3_unicycle_rendezvous.mp4
video_4_disconnected_graph.mp4
video_5_full_delay_vs_neighbor_only_delay.mp4
video_6_v3_edge_specific_delays.mp4
video_7_v3_time_varying_delay.mp4
video_8_v3_switching_geometric_graph.mp4
video_9_v3_collision_avoidance.mp4
video_10_v3_obstacle_avoidance.mp4
video_11_v3_packet_loss_50_percent.mp4
video_12_v3_dynamic_unicycle.mp4
video_13_v3_connectivity_maintenance_comparison.mp4
```

If MATLAB/Octave cannot directly create `.mp4` files, it saves frame folders such as:

```text
video_5_full_delay_vs_neighbor_only_delay_frames/
```

Each folder contains images named:

```text
frame_0001.png
frame_0002.png
frame_0003.png
...
```

---

## 13. Convert frames to MP4

A shell script is provided:

```bash
./convert_frames_to_mp4.sh p1_outputs_matlab/videos 15
```

The second argument is the frame rate. For example:

```bash
15
```

means 15 frames per second.

### Recommended robust version of the script

Use this version to avoid common macOS/QuickTime and `libx264` issues:

```bash
#!/bin/bash

VIDEO_DIR="$1"
FPS="${2:-15}"

for D in "$VIDEO_DIR"/*_frames; do
    [ -d "$D" ] || continue

    BASE=$(basename "$D" _frames)
    OUT="$VIDEO_DIR/$BASE.mp4"

    echo "Converting $D -> $OUT"

    rm -f "$OUT"

    ffmpeg -y \
      -framerate "$FPS" \
      -i "$D/frame_%04d.png" \
      -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2,format=yuv420p" \
      -c:v libx264 \
      -movflags +faststart \
      "$OUT"
done
```

Then run:

```bash
chmod +x convert_frames_to_mp4.sh
./convert_frames_to_mp4.sh p1_outputs_matlab/videos 15
```

The filter

```bash
-vf "pad=ceil(iw/2)*2:ceil(ih/2)*2,format=yuv420p"
```

is important because H.264 requires even frame dimensions. Some MATLAB-generated frames may have odd width or height, for example `1251x626`, which would otherwise fail with:

```text
width not divisible by 2
```

---

## 14. Troubleshooting

### 14.1 ffmpeg library error on macOS

If running `ffmpeg` produces an error like:

```text
Library not loaded: /usr/local/opt/x265/lib/libx265.xxx.dylib
```

then the Homebrew installation is inconsistent.

Fix:

```bash
brew update
brew reinstall x265
brew reinstall ffmpeg
```

Check:

```bash
which brew
which ffmpeg
brew --prefix
ffmpeg -version
```

For Intel Homebrew, the expected prefix is:

```text
/usr/local
```

For Apple Silicon Homebrew, the expected prefix is:

```text
/opt/homebrew
```

Do not mix the two installations.

---

### 14.2 MP4 file cannot be opened

If macOS says:

```text
Impossible to open the document
```

or the file appears as `0 KB`, then the conversion probably failed.

Delete the broken video:

```bash
rm -f p1_outputs_matlab/videos/video_5_full_delay_vs_neighbor_only_delay.mp4
```

Then reconvert from frames:

```bash
ffmpeg -y \
  -framerate 15 \
  -i p1_outputs_matlab/videos/video_5_full_delay_vs_neighbor_only_delay_frames/frame_%04d.png \
  -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2,format=yuv420p" \
  -c:v libx264 \
  -movflags +faststart \
  p1_outputs_matlab/videos/video_5_full_delay_vs_neighbor_only_delay.mp4
```

---

### 14.3 Video generation is too slow

Video generation is usually the slowest part of the project.

Recommended approach:

```matlab
MAKE_VIDEOS = false;
```

Run all numerical experiments first.

Then later:

```matlab
MAKE_VIDEOS = true;
```

Alternatively, generate only the most important videos:

```text
video_1_stable_2d_consensus
video_2_unstable_2d_consensus
video_3_unicycle_rendezvous
video_5_full_delay_vs_neighbor_only_delay
video_8_v3_switching_geometric_graph
video_9_v3_collision_avoidance
video_10_v3_obstacle_avoidance
video_13_v3_connectivity_maintenance_comparison
```

---

## 15. Main numerical results

The main reference topology is a ring graph with:

```text
N = 6
lambda_2 = 1
lambda_max = 4
tau_crit = pi / 8 ≈ 0.3927
```

Representative results:

| Experiment               | Result                                               |
| ------------------------ | ---------------------------------------------------- |
| 1D delayed consensus     | converges for (\tau = 0.5\tau_{crit})                |
| 2D delayed consensus     | rendezvous at initial centroid                       |
| Delay sweep              | unstable behavior appears above (\tau_{crit})        |
| Complete graph           | fastest convergence but lower delay margin           |
| Disconnected graph       | two local rendezvous points, no global rendezvous    |
| Neighbor-only delay      | more robust to large delays than full-state delay    |
| Unicycle rendezvous      | convergence achieved with nonlinear kinematics       |
| Packet loss              | convergence becomes slower as loss increases         |
| Collision avoidance      | compact cluster without exact zero disagreement      |
| Obstacle avoidance       | positive obstacle clearance                          |
| Dynamic unicycle         | convergence remains possible but slower              |
| Connectivity maintenance | preserving (\lambda_2>0) preserves global rendezvous |

---

## 16. Interpretation of key quantities

### (\lambda_2)

(\lambda_2), the algebraic connectivity, indicates whether the graph is connected and how strongly the network is connected.

* If (\lambda_2 > 0), the graph is connected.
* If (\lambda_2 = 0), the graph is disconnected.

In consensus, larger (\lambda_2) usually means faster convergence.

### (\lambda_{\max})

(\lambda_{\max}) determines the most restrictive delay mode in the full-state delay model.

A larger (\lambda_{\max}) reduces:

[
\tau_{crit} = \frac{\pi}{2\lambda_{\max}(L)}.
]

Therefore, dense graphs can converge faster but may tolerate less delay.

### (\tau_{crit})

(\tau_{crit}) is the theoretical critical delay of the full-state delayed consensus model.

* If (\tau < \tau_{crit}), the disagreement modes are stable.
* If (\tau > \tau_{crit}), instability is expected.

---

## 17. Report

The final integrated report is included as:

```text
Project_overview.pdf
```

The report contains:

* theoretical derivations;
* graph theory background;
* derivation of the delay threshold;
* all main experiments;
* v3 realistic extensions;
* figures and numerical tables;
* updated conclusions and limitations.

---

## 18. Reproducibility

The simulations use fixed random seeds where randomness is involved.

This makes the following experiments reproducible:

* random connected graphs;
* random geometric graphs;
* weighted graph sweeps;
* edge-specific delays;
* packet loss;
* switching graph cases.

To reproduce the full output:

1. open the project folder in MATLAB/Octave;
2. run `main.m`;
3. check `p1_outputs_matlab/`;
4. convert frame folders to `.mp4` if needed.

---

## 19. Known limitations

The project is simulation-based and has some limitations:

* the integration method is explicit Euler;
* collision avoidance and obstacle avoidance are implemented through repulsive heuristics;
* packet loss is modeled as independent random loss;
* sensor noise and state-estimation errors are not included;
* the dynamic unicycle model is still simplified;
* the connectivity maintenance strategy is numerical and heuristic;
* no real robot or high-fidelity robotic simulator is used.

These limitations are intentional: the project focuses on connecting graph theory, delayed consensus and multi-robot rendezvous in a clear and reproducible way.

---

## 20. Possible future work

Natural extensions include:

* using higher-order numerical integration methods;
* adding measurement noise and state estimation;
* studying directed graphs;
* studying stochastic and heterogeneous delays more formally;
* replacing repulsive fields with control barrier functions;
* implementing decentralized estimation of (\lambda_2);
* validating the controller in ROS 2 or a robotics simulator;
* comparing with passivity-based or MPC-based multi-robot controllers;
* adding event-triggered communication to reduce communication load.

---

## 21. How to cite this project internally

Suggested title:

```text
Rendezvous of Unicycles with Communication Delays
```

Suggested short description:

```text
A MATLAB/Octave simulation framework for delayed consensus and rendezvous of unicycle robots over graph-based communication networks, including spectral delay analysis and realistic extensions such as switching graphs, packet loss, obstacle avoidance and connectivity maintenance.
```

---

## 22. License

This repository is intended for academic use within the Control of Multi-Robot Systems course.

If reused or extended, please keep attribution to the original author and course project.

---

## 23. Author

**Samuele Civale**
Control of Multi-Robot Systems — Project 2026
Sapienza University of Rome
::: 

