# Project UAVs — Report 2: Crazyflie Motion Control and Planning

> **Course project** for Unmanned Aerial Vehicles 2025/2026.  
> Group 6

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Structure](#repository-structure)
3. [Part 1 — Linear Control (LQR)](#part-1--linear-control-lqr)
4. [Part 2 — Nonlinear Control (Lyapunov)](#part-2--nonlinear-control-lyapunov)
5. [Part 3 — ICUAS-Inspired Planning](#part-3--icuas-inspired-planning)
   - [Competition Scenario](#competition-scenario)
   - [Planning Algorithm](#planning-algorithm)
   - [Option A: 5 Free Relay Drones](#option-a-5-free-relay-drones)
   - [Option B: Shadow Drone + 4 Free Relays](#option-b-shadow-drone--4-free-relays)
   - [What Changes Between Options](#what-changes-between-options)
   - [How the Animation Works](#how-the-animation-works)
6. [Running the Code](#running-the-code)
7. [Videos](#videos)
8. [Report](#report)

---

## Project Overview

The project is divided into three independent parts:

| Part | Topic | Language |
|---|---|---|
| 1 | Linear Control — LQR (absolute state, nonlinear plant, error-space) | MATLAB |
| 2 | Nonlinear Control — Lyapunov stability-based controller | MATLAB |
| 3 | Motion planning and animation inspired by ICUAS 2026 competition scenario | Python |

Parts 1 and 2 are MATLAB simulations of a Crazyflie 2.1 (mass 29 g) tracking a spiral trajectory. **Part 3 is entirely separate**: it is a Python planning and animation exercise inspired by the ICUAS 2026 UAV Competition scenario, where drones must maintain a communication relay chain to a moving ground rover in an urban environment. The team did not participate in the competition — this is a course exercise.

---

## Repository Structure

```
Project-UAVs-23-code/
│
│  ── Parts 1 & 2: MATLAB ──────────────────────────────────────────────────────
│
├── init.m                             # Parameters, trajectory, gains
├── maio.m                             # Quick linear vs nonlinear comparison
├── class_1_4_LQR_Design.m            # Part 1 — LQR simulation (3 variants)
├── class_2_Lyapunov_Design.m         # Part 2 — Lyapunov simulation
│
└── src/
    ├── animateUAV.m                   # Visualisation helper: 3D playback of simulation results
    ├── plotResults.m                  # Plots + RMSE/ISE/ITAE metrics
    ├── lyapunovCtrl.m                 # Lyapunov control law
    ├── quad_dynamics_linear.m         # Linearised quadrotor ODE
    └── quad_dynamics_nonlinear.m      # Full nonlinear quadrotor ODE
│
│  ── Part 3: Python (animation + planning) ────────────────────────────────────
│
└── competition/
    ├── main.py                        # Option A — 5 free relay drones (animation)
    ├── main_sombra.py                 # Option B — shadow drone + 4 free relays (animation)
    ├── planeador.py                   # Planning core: union search, Dijkstra, Hungarian
    ├── mapa.py                        # Map builder: STL → pillars → visibility graph
    └── icuas26_1.stl                  # City world geometry (from ICUAS 2026 repo)
```

`.asv` files are MATLAB autosaves — safe to ignore.

---

## Part 1 — Linear Control (LQR)

**Entry point:** `class_1_4_LQR_Design.m`

Three LQR variants are compared on the same spiral trajectory:

| Variant | Plant | Control Law |
|---|---|---|
| Linear LQR | Linearised dynamics | `u = −K_lin · (x − x_d)` |
| Nonlinear LQR | Full nonlinear dynamics | Same gain `K_lin` on nonlinear plant |
| Error-Space LQR | Linearised dynamics | `u = −K_ES · e`, `e = x − x_d` |

All use forward Euler integration. The shared spiral reference (1 m radius, 2 revolutions, 10 s, +0.1 m/s ascent) and all gains are configured in `init.m`. `src/plotResults.m` generates comparative figures and performance tables; `src/animateUAV.m` plays back the simulation as a 3D visualisation.

---

## Part 2 — Nonlinear Control (Lyapunov)

**Entry point:** `class_2_Lyapunov_Design.m`

A Lyapunov tracking controller is derived from the candidate `V = eᵀe`:

```
u = −Kp · ep − Kv · ev + a_ff
```

`ep` and `ev` are position and velocity errors; `a_ff` is feedforward acceleration. The derivative:

```
V̇ = −epᵀ Kp ep − evᵀ (Kv − I) ev < 0   when Kv > I
```

guarantees global asymptotic stability. The controller runs alongside the LQR variants for direct comparison (RMSE, ISE, ITAE, peak error).

---

## Part 3 — ICUAS-Inspired Planning

> **This section is entirely independent from Parts 1 and 2.** All code here is Python. The animation, the planning algorithm, and the map files all belong to this part.

### Competition Scenario

The [ICUAS 2026 UAV Competition](https://github.com/larics/icuas26_competition) poses the following problem:

- A ground rover navigates an **urban obstacle field**
- A team of **Crazyflie drones** must maintain an unbroken **relay chain** from a fixed base station to the rover
- No direct base-to-rover link is allowed — the chain must pass through intermediate UAVs
- Evaluation: connectivity uptime, CBRNe threat identification, mission time

For this project the scenario is simplified: the rover path is fully known in advance, and the focus is exclusively on the **relay planning and animation** problem.

---

### Planning Algorithm

**Files:** `mapa.py`, `planeador.py`

**1. Map construction (`mapa.py`)**  
The city's 3D mesh (`icuas26_1.stl`) is sliced at z = 1 m to extract obstacle pillar footprints. Pillars are clustered and navigation nodes are generated around each at a 0.40 m clearance margin. A visibility graph connects all node pairs with clear line-of-sight.

**2. Relay corridor (`mapa.py → corredor_lazy`)**  
For each frame a lazy Dijkstra search finds the shortest node chain from base to the rover's current position. A sticky penalty (`PEN = 1.4`) keeps the corridor stable when the previous path is still valid, avoiding unnecessary replanning.

**3. Drone assignment (`planeador.py → planear`)**  
A union-search over a lookahead window (`L = 300` frames) selects target nodes covering the corridor now and in the near future (bracketing for upcoming turns). The Hungarian algorithm assigns drones to targets. An iterative Gauss-Seidel projection then enforces minimum separation (≥ 0.5 m) and obstacle clearance, while capping movement at `STEP = 0.1 m/frame` (v_max = 1.5 m/s).

---

### Option A: 5 Free Relay Drones

**File:** `main.py`

All **5 drones** are free agents. Each frame the planner places them along the relay corridor to span the full base-to-rover chain. No drone has a fixed role — the Hungarian assignment redistributes them every frame as needed.

```
Base ──[UAV 1]──[UAV 2]──[UAV 3]──[UAV 4]──[UAV 5]── Rover
```

The rover is the terminal node of the communication graph. If any link in the chain breaks, the relay is lost and the mission clock stops.

**Outputs:** `drone_relay.mp4` (4K UHD, 60 fps, ~40 s), `drone_relay_t22s.png`

---

### Option B: Shadow Drone + 4 Free Relays

**File:** `main_sombra.py`

One **shadow drone** is locked directly above the rover at all times — it tracks the rover's position exactly (rover speed 0.5 m/s << v_max 1.5 m/s). The remaining **4 relay drones** only need to connect the base to the shadow, whose position is always known.

```
Base ──[UAV 1]──[UAV 2]──[UAV 3]──[UAV 4]──[Shadow]
                                                ↕
                                              Rover
```

The shadow guarantees rover connectivity without planning. The 4 relays run the same union-search corridor algorithm, but the target is the shadow (not the rover directly).

**Output:** `drone_relay_sombra.mp4` (60 fps, ~40 s)

---

### What Changes Between Options

| Aspect | Option A — 5 Free Relays | Option B — Shadow + 4 Relays |
|---|---|---|
| **Rover connection** | Planned: one relay must keep LOS to rover | Guaranteed: shadow is always above rover |
| **Active drones** | 5, equal roles | 4 free + 1 shadow (distinct roles, distinct colours) |
| **Planning scope** | Full base → rover corridor | Reduced base → shadow corridor |
| **Planning complexity** | Higher — must always reach rover | Lower — terminal point is the shadow |
| **Single point of failure** | None (any relay can bridge to rover) | Shadow drone (if lost, rover link breaks) |
| **Max relay speed** | 1.5 m/s | Shadow: 0.5 m/s (rover-locked); relays: 1.5 m/s |
| **Disconnections** | 0 (validated) | 0 (validated) |

---

### How the Animation Works

Both animations use **Matplotlib `FuncAnimation`** rendered offline and exported via FFmpeg. They share the same architecture.

**Step 1 — Offline simulation**  
Before any rendering, `planear` / `planear_sombra` (in `planeador.py`) computes the full trajectory of every drone across all frames at 15 fps physics. This produces:
- `FD[t]` — list of drone positions at frame `t`
- `FR[t]` — communication graph: nodes, active edges, connected/broken flag

**Step 2 — Frame subsampling**  
The render selects every `stride`-th physics frame so the video duration matches the target (~40 s at 60 fps output).

**Step 3 — Per-frame update**  
Each rendered frame updates:
- **Relay links** — a `LineCollection` drawn in navy blue when the chain is connected, deep red when broken
- **Drone markers** — custom top-view quadrotor shape (body disc + 4 arms + 4 rotor discs at 45° increments) for relay drones; a diamond `◆` for the shadow drone in Option B
- **Rover marker** — rectangular body + 4 wheel circles, rotated to match the rover's instantaneous heading
- **Status overlays** — live simulation time, `● LINK ACTIVE / ✖ LINK BROKEN` badge, count of UAVs currently linked to the rover
- **Bottom panel** — legend and simulation info (frame, time, relay status)

**Option A** renders at **4K UHD** (3840×2160, 200 DPI). **Option B** is a lighter 1080p render used for faster iteration.

---

## Running the Code

### Parts 1 & 2 — MATLAB

Requirements: MATLAB R2021a+, Control System Toolbox.

```matlab
init                      % parameters and trajectory
class_1_4_LQR_Design      % Part 1: LQR comparison
class_2_Lyapunov_Design   % Part 2: Lyapunov comparison
```

`plotResults` and `animateUAV` are called automatically at the end of each script.

---

### Part 3 — Python

Requirements: Python 3.10+, `numpy`, `matplotlib`, `scipy`, `ffmpeg` (system).

```bash
cd competition/

# Option A — 5 free relay drones
python main.py
# → drone_relay.mp4, drone_relay_t22s.png

# Option B — shadow drone + 4 free relays
python main_sombra.py
# → drone_relay_sombra.mp4
```

On first run `mapa.py` builds the map from `icuas26_1.stl` and saves a cache (`mapa_cache.pkl`). Subsequent runs load the cache instantly.

---

## Videos

### Option A — 5 Free Relay Drones
> 5 Crazyflie drones planning their relay positions across the ICUAS 2026 city map to maintain connectivity from base to rover.

[hPython 2D Animation (5 relay drones)](https://github.com/user-attachments/assets/9959c9e4-abb2-4fd3-8a33-7f2756ada750)

<!-- Upload drone_relay.mp4 and replace RELAY5_VIDEO_ID -->

---

### Option B — Shadow Drone + 4 Free Relays
> Shadow drone locked above rover; 4 relay drones connect it back to the base.

[Python 2D Animation (4 relays + shadow drone)](https://github.com/user-attachments/assets/b3317299-f46f-4617-acd5-371c7bf41e3d)

<!-- Upload drone_relay_sombra.mp4 and replace SHADOW_VIDEO_ID -->

---- 

### Gazebo Simulation
> ROS 2 + Gazebo simulation of the ICUAS 2026 environment.

[Gazebo 3D Animation (5 relay drones)](https://github.com/user-attachments/assets/30812cee-1844-43c5-882f-f7da6eb45c84 )

<!-- Upload Gazebo video and replace GAZEBO_VIDEO_ID -->

---

## Report

Full report (PDF): **[INSERT REPORT LINK HERE]**

| Report Section | Code |
|---|---|
| Section 1 — Linear Control | `class_1_4_LQR_Design.m`, `src/` |
| Section 2 — Nonlinear Control | `class_2_Lyapunov_Design.m`, `src/lyapunovCtrl.m` |
| Section 3 — ICUAS Planning | `competition/main.py`, `main_sombra.py`, `planeador.py`, `mapa.py` |

---

*Report 2 — Group 6 — Unmanned Aerial Vehicles 2025/2026*
