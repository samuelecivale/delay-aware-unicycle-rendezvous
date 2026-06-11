# P1 — Rendezvous of Unicycles with Communication Delays

MATLAB/Octave plain implementation of the project:

**Control of Multi-Robot Systems — Project 2026**  
**P1: Rendezvous of unicycles with communication delays**

## How to run

Open MATLAB or GNU Octave in this folder and run:

```matlab
main
```

The script creates:

```text
p1_outputs_matlab/
├── figures/
├── tables/
└── videos/
```

## What is included

The project includes:

1. 1D delayed consensus;
2. 2D delayed consensus;
3. delay sweep;
4. topology comparison;
5. random connected graphs;
6. random geometric graphs;
7. disconnected graph failure case;
8. integration-step sensitivity;
9. unicycle rendezvous;
10. saturated vs unsaturated unicycle control;
11. weighted graphs with \(a_{ij}\neq 1\);
12. full-state delay vs neighbor-only delay;
13. unicycle comparison for the two delay models;
14. video generation.

## MATLAB / Octave notes

- The code is written in plain `.m` files.
- No Simulink is used.
- No toolboxes are required for the core simulations.
- MATLAB can generate `.mp4` videos using `VideoWriter`.
- If `VideoWriter` is not available, the animation functions save PNG frame sequences instead.

## Main models

Full-state delayed consensus:

\[
\dot{x}(t) = -Lx(t-\tau)
\]

Neighbor-only delayed consensus:

\[
\dot{x}(t) = -Dx(t) + Ax(t-\tau)
\]

Unicycle kinematics:

\[
\dot{x}_i = v_i \cos\theta_i,\quad
\dot{y}_i = v_i \sin\theta_i,\quad
\dot{\theta}_i = \omega_i
\]


## Octave graphics troubleshooting

If Octave crashes while saving figures, open `main.m` and set:

```matlab
MAKE_FIGURES = false;
MAKE_VIDEOS = false;
```

The simulations and CSV tables will still be generated.  
If you want figures, make sure Octave has a graphics toolkit installed, for example `gnuplot` or `qt`.

On macOS with Homebrew, useful commands may be:

```bash
brew install gnuplot
brew reinstall octave
```

Then run again:

```bash
octave main.m
```


## Where outputs are saved

The project saves outputs automatically:

```text
p1_outputs_matlab/
├── figures/   # PNG figures
├── tables/    # CSV tables
└── videos/    # MP4 videos or *_frames folders
```

In GNU Octave, `VideoWriter` is usually unavailable. In that case the code saves PNG frame folders such as:

```text
p1_outputs_matlab/videos/video_1_stable_2d_consensus_frames/
```

You can convert all frame folders to MP4 with:

```matlab
convert_all_frames_to_mp4('p1_outputs_matlab/videos', 15)
```

or from the shell:

```bash
./convert_frames_to_mp4.sh p1_outputs_matlab/videos 15
```

This requires `ffmpeg`.

## Neighbor-only delay tau critical

For the full-state delay model:

```math
\dot{x}(t) = -Lx(t-\tau)
```

the critical delay is:

```math
\tau_{crit} = \frac{\pi}{2\lambda_{\max}(L)}.
```

For the neighbor-only delay model:

```math
\dot{x}(t) = -Dx(t) + Ax(t-\tau)
```

there is no finite `tau_crit` of the same kind under the standard assumptions of an undirected connected graph with nonnegative weights. The disagreement dynamics are delay-independent stable, so in the ideal model:

```math
\tau_{crit}^{neighbor} = +\infty.
```

This is why the code uses the full-state `tau_crit` only as a reference scale when plotting the neighbor-only delay sweep.
