# Delay-Aware Rendezvous of Nonholonomic Multi-Robot Systems

![MATLAB](https://img.shields.io/badge/MATLAB-Multi--Robot%20Control-orange)
![Control](https://img.shields.io/badge/Control-Distributed-blue)
![Graph Theory](https://img.shields.io/badge/Graph%20Theory-Consensus-green)
![Robotics](https://img.shields.io/badge/Robotics-Networked%20Systems-purple)

A simulation framework for studying **distributed rendezvous and coordination of nonholonomic unicycle robots under communication delays**.

The project investigates how network latency interacts with graph topology, controller gain and robot dynamics, comparing different delay models and testing the controllers under increasingly realistic communication and coordination conditions.

---

## Problem

Each robot follows unicycle dynamics

\[
\dot{x}_i = v_i\cos\theta_i
\]

\[
\dot{y}_i = v_i\sin\theta_i
\]

\[
\dot{\theta}_i = \omega_i
\]

while its control input depends only on information exchanged through a communication graph.

The central question is:

> **How much communication delay can a distributed rendezvous controller tolerate before collective stability is lost?**

---

## Delay-Aware Consensus

For a linear consensus reference model

\[
\dot{p}(t)
=
-kLp(t-\tau)
\]

the theoretical critical delay is

\[
\tau_{crit}
=
\frac{\pi}
{2k\lambda_{max}(L)}
\]

where:

- \(k\) is the consensus gain;
- \(L\) is the graph Laplacian;
- \(\lambda_{max}(L)\) is its largest eigenvalue.

For the default six-agent ring topology:

\[
\lambda_{max}(L)=4
\]

which gives

\[
\tau_{crit}\approx0.3927\;s
\]

for unit consensus gain.

The simulations use this threshold as a theoretical reference and examine what happens below, near and above it.

---

## Communication Delay Models

Two delay models are compared.

### Full-State Delay

Each robot acts on a fully delayed consensus state:

\[
u(t)=-kLp(t-\tau)
\]

This model maps directly to the classical delayed-consensus stability analysis.

### Neighbor-Only Delay

Each robot knows its own current state but receives delayed measurements from its neighbors:

\[
u_i(t)
=
-k\sum_j a_{ij}
\left(
p_i(t)-p_j(t-\tau)
\right)
\]

This model is closer to a distributed robotic implementation, where local proprioception is immediate while network information arrives late.

---

## Nonholonomic Controllers

The delayed Cartesian interaction law cannot be directly applied to a unicycle.

The framework therefore contains two controller families.

### Potential-Field / Paper Controller

The interaction vector is projected onto the robot's admissible heading direction, respecting the nonholonomic structure.

### Vector-Field Controller

The delayed consensus law defines a desired planar velocity vector.

The unicycle then:

1. computes the desired motion direction;
2. aligns its heading with that direction;
3. advances with a bounded forward velocity.

This separates distributed coordination from low-level nonholonomic steering.

---

## Experimental Campaign

The simulator does more than reproduce a nominal rendezvous experiment.

It evaluates the controller under:

- subcritical and supercritical delays;
- different graph topologies;
- weighted, random and geometric graphs;
- disconnected networks;
- gain and timestep variations;
- actuator saturation;
- leader-follower coordination;
- rigid formations;
- measurement noise;
- packet loss;
- collision avoidance;
- obstacle avoidance;
- switching communication graphs;
- edge-dependent delays;
- time-varying delays.

The goal is to distinguish behavior caused by the **distributed control law** from behavior caused by network structure or communication imperfections.

---

## Graph-Theoretic Analysis

For every communication graph the framework evaluates spectral properties of the Laplacian, including:

\[
\lambda_2(L)
\]

which measures algebraic connectivity, and

\[
\lambda_{max}(L)
\]

which determines the theoretical full-state delay bound.

This makes it possible to directly relate topology to both convergence and delay robustness.

---

## Beyond Rendezvous

The same distributed framework is extended to more complex tasks:

### Leader-Follower Coordination

A designated robot influences the motion of the remaining agents.

### Rigid Formation Control

Agents converge while preserving desired pairwise offsets.

### Collision and Obstacle Avoidance

Repulsive terms are added to the nominal distributed interaction law.

### Switching Graphs

Communication edges change dynamically during the simulation.

These scenarios test whether the core delay-aware controller remains useful beyond ideal fixed-graph rendezvous.

---

## Running the Project

Open MATLAB in the repository directory and run:

```matlab
main
```

The main script runs the configured experiment suite and generates:

- trajectory plots;
- convergence metrics;
- graph statistics;
- CSV tables;
- optional videos.

Results are organized under:

```text
unicycle_only_outputs/
```

---

## Repository Structure

```text
.
├── main.m
├── unicycle_only_outputs/
│   ├── figures/
│   ├── tables/
│   ├── videos/
│   ├── paper/
│   └── vector_field/
├── README.md
└── ...
```

---

## What This Project Demonstrates

- Multi-Robot Systems
- Distributed Control
- Networked Robotics
- Consensus Algorithms
- Communication Delay
- Graph Laplacians
- Spectral Graph Theory
- Nonholonomic Control
- Formation Control
- Leader-Follower Systems
- Robustness Analysis
- Switching Networks
- Packet Loss and Measurement Noise

---

## Key Takeaway

Communication latency is not simply an implementation detail in distributed robotics: it fundamentally interacts with the spectrum of the communication graph.

This project connects the **theoretical delay stability boundary** of consensus systems with the behavior of nonlinear unicycle agents under realistic network imperfections, providing a systematic experimental study of when distributed coordination succeeds — and when it fails.
