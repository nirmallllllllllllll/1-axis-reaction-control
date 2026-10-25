# 1-axis-reaction-control
MatLab PID controlled Reaction wheel for CanSat and model rockets
# 1-Axis Roll Stabilization System for Model Rockets

<div align="center">
  <img src="https://img.shields.io/badge/MATLAB-Simulation-blue?logo=mathworks" alt="MATLAB" />
  <img src="https://img.shields.io/badge/Simulink-Control_Systems-blue?logo=simulink" alt="Simulink" />
  <img src="https://img.shields.io/badge/Control_Theory-Pole_Placement-success" alt="Control Theory" />
  <img src="https://img.shields.io/badge/Application-Aerospace_%7C_CanSat-critical" alt="Aerospace" />
</div>

## Overview
This repository contains the mathematical modeling, simulation scripts, and control system architecture for a 1-axis roll stabilization mechanism. Developed to meet the stringent constraints of a CanSat swarm-based micro-satellite experimental concept (including specifications relevant to events like the Bharatiya Antariksh Hackathon), this system stabilizes a model rocket's roll attitude without the use of aerodynamic fins. 

Instead of relying on atmospheric airflow, it utilizes an internal **Reaction Wheel (Ring Flywheel)**, exploiting the conservation of angular momentum to achieve perfect stabilization even at zero velocity or in a vacuum.

## Key Features
* **Rigorous Mathematical Modeling:** Full 1-DOF rotational Free Body Diagram (FBD) mapping to Laplace-domain transfer functions.
* **Pole Placement Tuning:** PID gains ($K_p$, $K_i$, $K_d$) are derived analytically for a critically damped response, bypassing heuristic "guess-and-check" methods.
* **Custom Simulink Architecture:** The controller is built from fundamental integrator and derivative blocks to prove control theory mechanics, avoiding "black box" standard PID blocks.
* **Type-2 Tracking Capabilities:** Zero steady-state error tracking for continuous ramp commands.

## System Architecture & Physical Constraints
To maintain realistic hardware limits and prevent motor saturation, the reaction wheel was optimized from a solid disk to a high-inertia outer ring.

| Parameter | Symbol | Value | Unit |
| :--- | :---: | :---: | :--- |
| **Rocket Roll Inertia** | $J_v$ | 0.0125 | $\text{kg}\cdot\text{m}^2$ |
| **Reaction Wheel Inertia** | $J_w$ | 0.002535 | $\text{kg}\cdot\text{m}^2$ |
| **Aerodynamic Damping** | $b$ | 0.002 | $\text{N}\cdot\text{m}\cdot\text{s/rad}$ |
| **Rocket Mass** | $M_v$ | ~3.5 | $\text{kg}$ |
| **Wheel Mass (Ring)** | $M_w$ | 0.6 | $\text{kg}$ |

## Control Theory & Math
The physical plant is modeled as an inertial system with aerodynamic friction:
$$J_v \ddot{\phi}(t) + b \dot{\phi}(t) = \tau_{motor}(t)$$

In the $s$-domain, the Plant Transfer Function $P(s)$ is:
$$P(s) = \frac{J_w}{s(J_v s + b)}$$

To achieve a critically damped response with a natural frequency of $\omega_n = 5 \text{ rad/s}$ (settling time ~1 second), the closed-loop characteristic equation is equated to the desired perfect polynomial $(s + 5)^3$. This yields the exact PID gains:
* **Derivative (Virtual Damper):** $K_d = 73.18$
* **Proportional (Virtual Spring):** $K_p = 369.82$
* **Integral (Error Memory):** $K_i = 616.37$

## Repository Structure
```text
├── docs/
│   ├── Project_Report_1_Axis_Roll_Stabilizer.pdf   # Full mathematical derivations
│   ├── FBD_Equivalent_System.png                   # Virtual spring-mass-damper diagrams
├── matlab_scripts/
│   ├── roll_stabilizer_plant.m                     # Calculates gains & plots step/root-locus
├── simulink_models/
│   ├── custom_pid_tracker.slx                      # Full block diagram with scope multiplexing
└── README.md
