# Delay-Aware Unicycle Rendezvous

MATLAB/Octave implementation of delayed consensus and rendezvous control for multi-robot systems.

This project studies how communication delays affect distributed rendezvous algorithms for single-integrator agents and unicycle robots. The implementation starts from the classical delayed consensus model, derives the critical delay bound, compares different graph topologies, extends the analysis to weighted communication graphs, and introduces an alternative neighbor-only delay model where each robot knows its own current state but receives delayed information from its neighbors.

## Project Overview

The main objective is to analyze the rendezvous problem for a team of robots communicating over an undirected graph.

The baseline delayed consensus model is

```math
\dot{x}(t) = -Lx(t-\tau),
```

where (L) is the graph Laplacian and (\tau) is a constant communication delay.

For this model, the critical delay is

```math
\tau_{crit} = \frac{\pi}{2\lambda_{\max}(L)}.
```

The project also studies a more communication-realistic model:

```math
\dot{x}(t) = -Dx(t) + Ax(t-\tau),
```

where each robot uses its own current state and delayed neighbor states. Under standard assumptions of connected undirected graphs with nonnegative weights, this model is delay-independent stable in the disagreement subspace, so no finite critical delay exists in the ideal continuous-time model.

## Main Features

* 1D delayed consensus simulation.
* 2D rendezvous with delayed communication.
* Critical delay analysis based on Laplacian eigenvalues.
* Delay sweep below, near, and above the stability threshold.
* Comparison between path, ring, star, and complete graphs.
* Random connected graph experiments.
* Random geometric graph experiments.
* Disconnected graph case showing local rendezvous only.
* Numerical integration-step sensitivity analysis.
* Weighted graph experiments with nonuniform (a_{ij}).
* Unicycle rendezvous with conversion from consensus velocity to (v_i,\omega_i).
* Saturated and unsaturated unicycle control comparison.
* Full-state delay versus neighbor-only delay comparison.
* Automatic generation of figures, CSV tables, and videos.

## Models

### Full-State Delayed Consensus

```math
\dot{x}_i(t)
=
-\sum_{j\in\mathcal{N}_i}
a_{ij}
\left(
x_i(t-\tau)-x_j(t-\tau)
\right)
```

Matrix form:

```math
\dot{x}(t)=-Lx(t-\tau)
```

Critical delay:

```math
\tau_{crit} = \frac{\pi}{2\lambda_{\max}(L)}
```

### Neighbor-Only Delayed Consensus

```math
\dot{x}_i(t)
=
-\sum_{j\in\mathcal{N}_i}
a_{ij}
\left(
x_i(t)-x_j(t-\tau)
\right)
```

Matrix form:

```math
\dot{x}(t)=-Dx(t)+Ax(t-\tau)
```

This model does not use the same critical delay formula as the full-state delayed model. Under connected undirected graphs with nonnegative weights, the disagreement dynamics are delay-independent stable.

### Unicycle Model

Each robot is modeled as

```math
\dot{x}_i = v_i\cos\theta_i,
\qquad
\dot{y}_i = v_i\sin\theta_i,
\qquad
\dot{\theta}_i = \omega_i.
```

The consensus velocity is converted into unicycle commands through

```math
\theta_{d,i} = \operatorname{atan2}(u_{y,i},u_{x,i}),
```

```math
e_{\theta,i} = \operatorname{wrap}(\theta_{d,i}-\theta_i),
```

```math
\omega_i = k_\theta e_{\theta,i},
\qquad
v_i = k_v\|u_i\|\cos(e_{\theta,i}).
```

## Repository Structure

```text
.
├── main.m
├── README.md
├── adjacency_matrix.m
├── laplacian_matrix.m
├── graph_spectral_info.m
├── neighbor_only_delay_info.m
├── simulate_delayed_consensus_1d.m
├── simulate_delayed_consensus_2d.m
├── simulate_unicycle_rendezvous.m
├── simulate_neighbor_only_delay_consensus_1d.m
├── simulate_neighbor_only_delay_consensus_2d.m
├── simulate_unicycle_neighbor_only_delay.m
├── plot_*.m
├── save_*.m
├── p1_outputs_matlab
│   ├── figures
│   ├── tables
│   └── videos
└── convert_frames_to_mp4.sh
```

## How to Run

Open MATLAB or GNU Octave in the project folder and run:

```matlab
main
```

The script creates or updates:

```text
p1_outputs_matlab/
├── figures/
├── tables/
└── videos/
```

## Output

The repository includes generated outputs:

* PNG figures in `p1_outputs_matlab/figures/`;
* CSV result tables in `p1_outputs_matlab/tables/`;
* MP4 videos in `p1_outputs_matlab/videos/`.

The main generated videos are:

```text
video_1_stable_2d_consensus.mp4
video_2_unstable_2d_consensus.mp4
video_3_unicycle_rendezvous.mp4
video_4_disconnected_graph.mp4
video_5_full_delay_vs_neighbor_only_delay.mp4
```

## MATLAB / Octave Notes

The code is written in plain MATLAB/Octave style.

No Simulink is required.

In GNU Octave, `VideoWriter` may be unavailable. In that case, the code can save frame folders instead of MP4 videos. These frames can be converted to MP4 using:

```bash
./convert_frames_to_mp4.sh p1_outputs_matlab/videos 15
```

or from MATLAB/Octave:

```matlab
convert_all_frames_to_mp4('p1_outputs_matlab/videos', 15)
```

## Key Results

For the ring graph with (N=6), the Laplacian eigenvalues are approximately

```math
\lambda(L)=\{0,1,1,3,3,4\}.
```

Therefore,

```math
\lambda_2=1,
\qquad
\lambda_{\max}=4,
```

and the full-state delay critical value is

```math
\tau_{crit}=\frac{\pi}{2\cdot 4}=\frac{\pi}{8}\approx 0.3927.
```

The simulations confirm that:

* the full-state delayed consensus converges for (\tau < \tau_{crit});
* convergence becomes slower and more oscillatory near (\tau_{crit});
* the system becomes unstable above (\tau_{crit});
* graph topology affects convergence through (\lambda_2);
* delay robustness depends on (\lambda_{\max});
* disconnected graphs produce local rendezvous, not global rendezvous;
* weighted graphs modify the Laplacian spectrum and therefore the dynamics;
* unicycle robots reach rendezvous, although the final point may differ from the initial centroid;
* the neighbor-only delay model behaves differently from the full-state delayed model.

## Author

Samuele Civale
MSc in Artificial Intelligence and Robotics
Sapienza University of Rome

