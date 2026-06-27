# Project UAVs — 23 UAVs ICUAS 2026

> **MATLAB simulation and control design** for quadrotor UAVs developed for the **ICUAS 2026** competition.  
> This repository contains all control code, dynamics models, animation tooling, and results-plotting infrastructure that are part of the "23 UAVs" project work.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Structure](#repository-structure)
3. [Key Files — ICUAS 2026](#key-files--icuas-2026)
4. [How the Animation Works](#how-the-animation-works)
5. [Controller Designs](#controller-designs)
6. [Running the Simulation](#running-the-simulation)
7. [Videos](#videos)
8. [Report](#report)

---

## Project Overview

This project implements and compares three quadrotor control strategies on a spiral reference trajectory:

| Controller | Model | Method |
|---|---|---|
| Linear LQR | Linearised quadrotor | State-feedback |
| Nonlinear LQR | Full nonlinear quadrotor | State-feedback |
| Error-Space LQR | Linearised quadrotor | Error-state formulation |
| Lyapunov | Nonlinear quadrotor | Lyapunov stability theory |

The drone modelled is a **Crazyflie** (mass 29 g). The reference trajectory is a **spiral** of 1 m radius completing 2 revolutions over 10 s while ascending at 0.1 m/s.

---

## Repository Structure

```
Project-UAVs-23-code/
│
├── init.m                        # Entry point — drone parameters, trajectory, controller gains
├── maio.m                        # Standalone script: nonlinear vs linear comparison
├── class_1_4_LQR_Design.m        # Main simulation: LQR variants (linear / nonlinear / error-space)
├── class_2_Lyapunov_Design.m     # Main simulation: Lyapunov controller
│
└── src/
    ├── animateUAV.m              # 3D animation of UAV bodies in flight
    ├── plotResults.m             # Comparative plots + performance metrics
    ├── lyapunovCtrl.m            # Lyapunov control law
    ├── quad_dynamics_linear.m    # Linearised quadrotor dynamics (ODE RHS)
    └── quad_dynamics_nonlinear.m # Full nonlinear quadrotor dynamics (ODE RHS)
```

`.asv` files are MATLAB autosaves and can be ignored.

---

## Key Files — ICUAS 2026

### `init.m` — System Initialisation
Configures all shared parameters used across simulations:
- Drone physical constants: mass `m = 0.029 kg`, gravity `g = 9.81 m/s²`, damping `b = 0.001`
- LQR weighting matrices `Q` and `R` for each controller variant
- Lyapunov gains `Kp` and `Kv`
- Spiral trajectory generation (position, velocity, and acceleration sampled analytically)
- Plot aesthetics: Times New Roman, LaTeX interpreters, custom colour palette

### `class_1_4_LQR_Design.m` — LQR Simulation
Runs three parallel control loops over the same spiral trajectory:
1. **Linear LQR** — feedback law `u = −K_lin · (x − x_d)` integrated with `quad_dynamics_linear`
2. **Nonlinear LQR** — same gain structure applied to `quad_dynamics_nonlinear`
3. **Error-Space LQR** — error state `e = x − x_d` fed directly into `u = −K_ES · e`

All loops use **forward Euler** integration with timestep `dt`. Results are collected in a `models` struct and passed to `plotResults` and `animateUAV`.

### `class_2_Lyapunov_Design.m` — Lyapunov Simulation
Applies the Lyapunov-based controller alongside the LQR variants for direct comparison. The control law is:

```
u = −Kp · ep − Kv · ev + u_ff
```

where `ep` and `ev` are position and velocity errors and `u_ff` is the feedforward acceleration. Stability is guaranteed by the Lyapunov candidate `V = eᵀe`, which is negative-definite when `Kv > I`.

### `src/quad_dynamics_nonlinear.m` — Nonlinear Model
Converts control acceleration commands into thrust and Euler angles, builds the full ZYX rotation matrix, and returns the 6-DOF state derivative `[ṗ; v̇]` in the inertial frame.

### `src/quad_dynamics_linear.m` — Linearised Model
Same interface as the nonlinear model but uses small-angle approximations, making it suitable for LQR gain synthesis around the hover equilibrium.

### `src/plotResults.m` — Results & Metrics
Generates all figures used in the ICUAS 2026 report:
- 3D trajectory comparison
- Position and velocity time-series (per axis)
- Control inputs with hardware saturation limits (±10° angles, 0.588 N thrust)
- Per-axis tracking error with 3σ bounds
- Error norm over time
- Performance table: **RMSE, ISE, ITAE, peak error** for every controller

---

## How the Animation Works

The animation is produced by `src/animateUAV.m`. It receives:

| Argument | Contents |
|---|---|
| `models` | Struct array — one entry per controller, with state history `x`, control history `u`, colour, and name |
| `t` | Time vector |
| `ref` | Reference trajectory struct with position array `ref.p` |

### Step-by-step

**1. Scene setup**  
A single 3D axes is created with equal aspect ratio. Each model gets two graphics handles: a **body line** (solid, controller colour) and a **trajectory trail** (dashed).  The reference path is drawn once as a static black dashed line.

**2. UAV geometry**  
The quadrotor is represented as a **cross of four arms** of length 0.25 m, stored as an 8-point line in body frame:

```
body = [ 0   0  0;   +arm 0 0;   0 0 0;   -arm 0 0;
         0   0  0;   0  +arm 0;  0 0 0;   0  -arm 0 ]
```

**3. Attitude reconstruction (per frame)**  
At each timestep `i` (sampled every 5 steps for speed), the function reconstructs roll `φ` and pitch `θ` from the control input:

- **Lyapunov**: acceleration vector `a = u + [0; 0; mg]` → thrust direction `b₃ = a/‖a‖` → exact angles via `atan2`
- **LQR**: small-angle linearisation — `φ = u₁/T`, `θ = −u₂/T` where `T = u₃ + mg`

Yaw `ψ` is fixed at zero (the control design assumption).

**4. Rotation and translation**  
The ZYX rotation matrix `R = Rz · Ry · Rx` is applied to every arm vertex, then the drone position `p(i)` is added:

```matlab
P = (R * body')' + p;
```

**5. Graphics update**  
`set(hBody, 'XData', ..., 'YData', ..., 'ZData', ...)` updates the body cross in place.  
`set(hTraj, ...)` extends the trajectory trail up to timestep `i`.  
`drawnow limitrate` + `pause(0.06 s)` paces the playback.

The result is a real-time 3D animation showing every controller's UAV simultaneously — each in its own colour — banking and pitching correctly as it follows the spiral.

---

## Controller Designs

### LQR (Linear Quadratic Regulator)

The state vector is `x = [px, py, pz, vx, vy, vz]ᵀ`. The gain matrix `K` is computed via MATLAB's `lqr()` on the linearised double-integrator model. Three formulations are compared:

- **Linear** — applies `K` to the full linearised plant
- **Nonlinear** — applies the same `K` to the nonlinear plant (robustness test)
- **Error-space** — tracks the error state directly, equivalent to an integral-like formulation around the reference

### Lyapunov Controller

Designed using the candidate function `V = eᵀe`. The control law:

```
u = −Kp · ep − Kv · ev + a_ff
```

guarantees `V̇ < 0` when `Kv > I`, providing **global asymptotic stability** for the position error dynamics. Feedforward acceleration `a_ff` cancels the known trajectory curvature.

---

## Running the Simulation

**Requirements:** MATLAB R2021a or later (no additional toolboxes required beyond Control System Toolbox for `lqr()`).

```matlab
% 1. Initialise parameters and trajectory
init

% 2a. Run LQR comparison (linear / nonlinear / error-space)
class_1_4_LQR_Design

% 2b. Run Lyapunov comparison
class_2_Lyapunov_Design

% 3. (Optional) Quick nonlinear vs linear comparison
maio
```

`plotResults` and `animateUAV` are called automatically at the end of each simulation script.

---

## Videos

### Animation (MATLAB)
> 3D MATLAB animation showing all controllers tracking the spiral trajectory simultaneously.

https://github.com/user-attachments/assets/ANIMATION_VIDEO_ID

<!-- Replace ANIMATION_VIDEO_ID with the actual GitHub asset ID after uploading the video from /secretaria -->

---

### Gazebo Simulation
> ROS + Gazebo flight simulation of the Crazyflie executing the spiral trajectory under LQR control.

https://github.com/user-attachments/assets/GAZEBO_VIDEO_ID

<!-- Replace GAZEBO_VIDEO_ID with the actual GitHub asset ID after uploading the video from /secretaria -->

---

## Report

This repository is supplementary material for the ICUAS 2026 paper:

> **[Paper title — to be filled in]**  
> *[Authors]*  
> International Conference on Unmanned Aircraft Systems (ICUAS) 2026

Full report: **[INSERT REPORT LINK HERE]**

The code, videos, and figures in this repository correspond directly to Section [X] (Simulation Results) of the report.

---

*Developed as part of the 23 UAVs project.*
