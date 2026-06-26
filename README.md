# Control of Multi-Robot Systems — Unicycle Rendezvous with Communication Delay

Project 2026. Study of the **rendezvous (consensus)** of a group of **unicycle** robots
under **communication delay**, with analysis of the critical delay `tau_crit` and a set of
extensions (formations, leader-follower, obstacle avoidance, switching graphs, robustness
to noise and packet loss).

All the code is contained in a single self-contained MATLAB file:
`main_unicycle.m` (the `main` function plus all local helper functions).

---

## 1. Model

Each agent $i$ is a unicycle:

$$
\dot{x}_i = v_i \cos\theta_i, \qquad
\dot{y}_i = v_i \sin\theta_i, \qquad
\dot{\theta}_i = \omega_i
$$

The desired Cartesian velocity is produced by a **delayed consensus** law and then realized
on the unicycle. Two delay models are included:

- **full_state:** $\ u(t) = -kLp(t - \tau)$
- **neighbor_only:** $\ u_i(t) = -k \sum_{j} a_{ij}\\bigl( p_i(t) - p_j(t - \tau) \bigr)$

where $L$ is the Laplacian of the communication graph and $k$ is the consensus gain.

### Critical delay

For the linear full-state single-integrator reference model:

$$
\tau_{\mathrm{crit}} = \frac{\pi}{2k\lambda_{\max}(L)}
$$

This is the **reference value** used throughout the project. In the unicycle case the
nonlinear heading dynamics, saturation and numerical integration may make the observed
behavior slightly different around the boundary.

---

## 2. Controllers

The main runs the study with two controller families, selectable via `CONTROLLER_TYPE`:

- **`paper`** (default): nonholonomic controller inspired by the rendezvous/formation
  papers based on potential fields. The delayed interaction force is projected onto the
  admissible direction of motion of the unicycle.
- **`vector_field`**: the previous project controller. The delayed consensus field produces
  a desired Cartesian velocity, which is then tracked by a heading controller.

With `RUN_BOTH_CONTROLLERS = true` (current setting) the entire experiment suite is executed
**twice**, once per controller, saving the outputs in separate folders.

---

## 3. Main parameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `N` | 6 | number of robots (base case) |
| `params.k` | 1.0 | consensus gain |
| `params.kv` | 1.0 | linear velocity gain |
| `params.ktheta` | 3.0 | heading gain |
| `params.vmax` | 1.5 | linear velocity saturation |
| `params.omegamax` | 4.0 | angular velocity saturation |
| `params.saturate` | true | enables command saturation |
| `T` | 35.0 s | time horizon (some tests use different values) |
| `dt` | 0.02 s | integration step |
| `eps_convergence` | 1e-2 | threshold for convergence time |

Base graph: 6-node ring, with $\lambda_{\max}(L) = 4$, $\lambda_2(L) = 1$,
giving $\tau_{\mathrm{crit}} \approx 0.3927\ \mathrm{s}$.

---

## 4. How to run

Requirements: **MATLAB R2025** (tested). Video writing uses `VideoWriter`
(MPEG-4 profile, with Motion JPEG AVI fallback).

From the project folder:

```matlab
main_unicycle
```

Useful flags at the top of the file:

- `MAKE_FIGURES` — generate and save the PNG figures
- `MAKE_VIDEOS` — generate the trajectory videos (see Section 6)
- `RUN_BOTH_CONTROLLERS` — run both `paper` and `vector_field`
- `CLOSE_FIGURES_AFTER_SAVE` — close figures after saving

---

## 5. Output structure

Outputs are separated per controller:

```
unicycle_only_outputs/
├── paper/
│   ├── figures/   PNGs of trajectories, disagreement curves, summaries
│   ├── tables/    CSVs with the metrics of each test
│   └── videos/    MP4 animations (selected tests only)
└── vector_field/
    ├── figures/
    ├── tables/
    └── videos/
```

> Note: any `figures/`, `tables/`, `videos/` folders directly under
> `unicycle_only_outputs/` (outside `paper/` and `vector_field/`) are leftovers from old
> runs and can be deleted.

---

## 6. Videos

To keep the outputs lean (useful for the presentation), videos are generated **only for the
most significant tests**, for both controllers:

| ID | Content | File |
|----|---------|------|
| U1 | Baseline full-state rendezvous (converges) | `u01_full_state.mp4` |
| U3 | Supra-critical comparison at $1.2\,\tau_{\mathrm{crit}}$: full-state diverges | `u03_full_state_supracritical.mp4` |
| U3 | Supra-critical comparison at $1.2\,\tau_{\mathrm{crit}}$: neighbor-only stable | `u03_neighbor_only_supracritical.mp4` |
| U8 | Disconnected graph: cluster rendezvous | `u08_disconnected_full_state.mp4`, `u08_disconnected_neighbor_only.mp4` |
| U15 | Leader-follower with moving target | `u15_leader_follower.mp4` |
| U16 | Rigid formation | `u16_rigid_formation.mp4` |
| U19 | Obstacle and collision avoidance | `u19_obstacle_collision_avoidance.mp4` |
| U20 | Time-varying geometric (switching) graph | `u20_switching_graph.mp4` |

All other tests produce figures and tables only.

### Automatic skip

`save_unicycle_animation_local` skips generation if the video (or its `.avi` fallback)
**already exists** on disk. This lets you re-run the main without regenerating videos that
were already produced. If writing a video fails, the partial file is removed so it can be
recreated on the next run.

---

## 7. List of tests

### Baseline study and delay analysis

- **U1** — Baseline full-state: rendezvous with sub-critical delay.
- **U2** — Baseline neighbor-only at the same delay.
- **U3** — Sweep of $\tau$ (from $0$ to $2\,\tau_{\mathrm{crit}}$): full-state vs neighbor-only. Shows the
  divergence of full-state above the threshold and the higher robustness of neighbor-only.
- **U4** — Topology comparison (path, ring, star, complete).
- **U5** — Ring graph with random weights.
- **U6** — Random connected graphs (statistics over several trials).
- **U7** — Random geometric graphs, varying the communication radius.
- **U8** — Disconnected graph: cluster rendezvous instead of global rendezvous.
- **U9** — Saturated vs unsaturated commands.
- **U10** — Sensitivity to the integration step $dt$.
- **U11** — Effect of the gain $k$ on $\tau_{\mathrm{crit}}$.
- **U12** — Sensitivity to the initial heading.
- **U13** — Complete graphs, varying size and edge weight.

### Extensions

- **U14** — Rendezvous to an arbitrary target point.
- **U15** — Leader-follower with a moving target.
- **U16** — Rigid formation (consensus on shifted coordinates).
- **U17** — Robustness to noise (measurement, command, process).
- **U18** — Robustness to packet loss.
- **U19** — Obstacle avoidance + inter-robot safety (continuous repulsive terms).
- **U20** — Time-varying geometric graph with monitoring of $\lambda_2(L(t))$.
- **U21** — Edge-dependent delays (different $\tau_{ij}$ for each edge).
- **U22** — Time-varying sinusoidal delays $\tau(t)$, including a profile that crosses the threshold.
- **U23** — Clearance formation via attractive/repulsive potential fields.
- **U24** — Switching obstacle avoidance (local tangential controller).
- **U25** — Delay initial-history protocol (constant vs zero).
- **U26** — Summary table of the extensions.

---

## 8. Tables (CSV)

Each test writes one or more CSVs in `tables/` with the main metrics, including: final and
maximum disagreement (RMS), convergence time, final centroid, mean path length, and — where
relevant — $\tau$, $\tau/\tau_{\mathrm{crit}}$, $\lambda_2$, $\lambda_{\max}$, practical-stability flag. File
names follow the scheme `uNN_<description>.csv`.

---

## 9. Metrics

- **RMS disagreement:** root-mean-square deviation of the positions from the centroid;
  measures how tightly the group is gathered.
- **Convergence time:** first instant beyond which the disagreement stays below
  $\varepsilon_{\mathrm{conv}}$ until the end.
- **Practical stability:** flag based on the final disagreement value and the slope of the
  tail of the curve.
- **Mean path length:** total distance traveled, averaged over the robots.

---

## 10. Notes

- The random number generator seed is reset in a controlled way (`safe_rng_local`) to make
  the results reproducible across runs.
- The `tau_crit` values used in the tests always refer to the linear full-state model; they
  serve as a reference threshold for the unicycle and neighbor-only cases as well, i.e.
  $\tau_{\mathrm{crit}} = \pi / \bigl(2\,k\,\lambda_{\max}(L)\bigr)$.
