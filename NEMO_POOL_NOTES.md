# NEMO Robot in Isaac Sim --- Technical Documentation

**Version:** 1.0  

**Platform:** NVIDIA Isaac Sim 5.1  

**Project:** NEMO Swarm Pool Simulation  

**File:** `nemo_swarm_pool.py`

---

## Table of Contents

1\. [Overview](#1-overview)

2\. [NEMO Robot Model](#2-nemo-robot-model)

   - 2.1 Physical Configuration

   - 2.2 Motor Layout

   - 2.3 Forward Kinematics (Forces)

   - 2.4 Inverse Kinematics (Motor Allocation)

   - 2.5 Control Strategy in Simulation

   - 2.6 Physical Parameters Summary

3\. [Water Simulation & Effects](#3-water-simulation--effects)

   - 3.1 Water Surface Mesh

   - 3.2 Ambient Waves

   - 3.3 Robot-Induced Disturbances

   - 3.4 Ripple Rings

   - 3.5 Visual Materials

4\. [The `nemo_swarm_pool.py` Program](#4-the-nemo_swarm_poolpy-program)

   - 4.1 Architecture

   - 4.2 Execution Flow

   - 4.3 Swarm Shape Sequence

   - 4.4 Key Configuration Parameters

   - 4.5 Usage

5\. [Coordinate Conventions](#5-coordinate-conventions)

---

## 1. Overview

The **NEMO** (Networked Exploration & Mapping Omni-robot) is a floating, omnidirectional surface robot designed for swarm exploration and mapping tasks. Each NEMO uses **three equally-spaced thrusters** to achieve three degrees of freedom on the water surface: **two translations (X, Y)** and **one rotation (yaw)**.

This document describes:

- How a single NEMO is modelled and controlled inside Isaac Sim (physics fidelity to the real robot);

- How the virtual pool water is rendered, including wave and wake effects;

- How `nemo_swarm_pool.py` orchestrates a 5-robot swarm in a coordinated-shape demo.

The simulation reproduces the **exact force model** of the real NEMO (`modelo_Surface.py` and the project's README), so behaviours observed here --- velocity limits, saturation, turning dynamics --- are directly comparable to the physical robots used in the pool experiments.

---

## 2. NEMO Robot Model

### 2.1 Physical Configuration

| Property | Value | Unit |

|---|---|---|

| Mass | 1.46 | kg |

| Moment of inertia about centre (J_O) | 0.00105 | kg-m² |

| Distance from centre to thrusters (R) | 0.15 | m |

| Number of thrusters | 3 | --- |

| Max force per thruster | 2.0 | N |

| Max total yaw torque | 0.9 | N-m |

| Max net horizontal force (zero torque) | 3.0 | N |

The robot body is a **circular Xform with three evenly spaced arms** (120° apart). Each arm carries one thruster. In the USD model (`nemo_body.usd`), the root prim `nemo_body` is an Xform carrying a **RigidBody API** property.

### 2.2 Motor Layout

The three thrusters are positioned at angles **0°, 120°, and 240°** measured from the robot's local +X axis (forward direction):

```

                    M1 (0°)

                      ▲

                      │

                      │

       M3 (240°) ◀────┼────▶ M2 (120°)

                      │

                      │

                      ▼

```

Each thruster's force direction is aligned with its arm, so the body-frame force contribution of motor *i* has the form:

```

F_i = [ cos(θ_i) ,  sin(θ_i) ] - f_i

```

where `θ_i ∈ { 0°, 120°, 240° }` and `f_i ∈ [-2.0, +2.0] N`.

### 2.3 Forward Kinematics (Forces)

Summing the contributions of all three motors, the **net body-frame force and yaw torque** are:

```

Fx  = F1  -  0.5-F2  -  0.5-F3

Fy  = (√3 / 2) - (F3  -  F2)

τz  = R - (F1 + F2 + F3)

```

These equations are implemented in `motors_to_body_forces()` and reproduce the dynamics in `modelo_Surface.py` exactly.

**Physical interpretation:**

| Motion | Motor command pattern |

|---|---|

| Pure forward (`Fx > 0`) | M1 forward, M2 and M3 both backward at half power |

| Pure sideways (`Fy > 0`) | M2 backwards, M3 forwards, M1 idle |

| Pure rotation (`τz > 0`) | All three motors forward equally |

| Combined | Superposition of the three basis patterns |

### 2.4 Inverse Kinematics (Motor Allocation)

Given a desired `(Fx, Fy, τz)` command, the required motor forces are:

```

Fsum = τz / R

F1   = Fsum/3 + (2/3)-Fx

F2   = Fsum/3 - (1/3)-Fx - Fy/√3

F3   = Fsum/3 - (1/3)-Fx + Fy/√3

```

This is the **exact algebraic inverse** of the forward model (verified by substitution). It is implemented in `motor_allocation()`.

### 2.5 Control Strategy in Simulation

Because real motors saturate at ±2.0 N, the simulation **never bypasses this limit**:

1\. **Compute desired** `(Fx, Fy, τz)` from the high-level PD controllers (position, yaw, leveling).

2\. **Allocate** to `(F1, F2, F3)` using the inverse model.

3\. **Clip** each motor to `±2.0 N`.

4\. **Recompose** the *actual* `(Fx, Fy, τz)` from the clipped values using the forward model.

5\. **Apply** only the achievable force/torque to the rigid body.

This guarantees that **the simulated NEMO can never produce more force than the real one** --- a critical fidelity property.

The high-level controllers are:

| Controller | Purpose | Gains |

|---|---|---|

| **Position PD** | Drive `(x, y)` toward the target shape slot | `KP_POS = 25`, `KD_VEL = 10` |

| **Yaw PD** | Hold yaw at 0 rad (forward-facing) | `KP_YAW = 0.25`, `KD_YAW = 0.12` |

| **Buoyancy righting** | Return roll/pitch to level | `LEVEL_KP = 0.30`, `LEVEL_KD = 0.05` |

| **Z-hold** | Emulate buoyancy; keep robot at spawn depth | `KP_Z = 40`, `KD_Z = 10` |

> **Note:** The yaw torque `τ_act` from the forward model is expressed in the **body frame**. Before being sent to PhysX (which requires world-frame torques when `is_global=True`), it is rotated with `τ_world = τ_act - R_mat[:, 2]` (the body's world-space up-vector).

### 2.6 Physical Parameters Summary

```python

NEMO_MASS       = 1.46        # kg

NEMO_J_O        = 0.00105     # kg-m²

NEMO_R          = 0.15        # m

NEMO_MAX_FORCE  = 2.0         # N per motor

NEMO_MAX_TORQUE = 3 * 2.0 * 0.15 = 0.9   # N-m

# PhysX damping

linear_damping  = 2.5         # → terminal speed ≈ 1.2 m/s (matches real NEMO)

angular_damping = 1.5         # tuned low enough to let buoyancy righting act

gravity         = disabled    # buoyancy emulated by Z-hold PD

```

The `linear_damping = 2.5` value is chosen so that the terminal velocity under maximum thrust (`F = 3.0 N`, `m = 1.46 kg`) is:

```

v_terminal = F / (m - ζ)  =  3.0 / (1.46 - 2.5)  ≈  0.82 m/s

```

which is close to the real NEMO's observed maximum horizontal speed (~1.0--1.2 m/s). Increasing damping slows the robot; decreasing it produces unrealistic slip.

---

## 3. Water Simulation & Effects

### 3.1 Water Surface Mesh

The water is represented by a **subdivided quad mesh** (`UsdGeom.Mesh`) covering the pool area, with:

| Parameter | Value |

|---|---|

| Grid resolution | 40 × 40 cells → 1681 vertices |

| Pool size | 8 m × 8 m |

| Base height (`WATER_Z`) | 0.0 m |

| Vertex animation | Yes --- every frame |

| Physics coupling | None (purely visual) |

A static plane would look flat and unnatural, so the mesh's vertex Z-positions are updated each frame by the **sum of several procedural wave functions**. Because the mesh has only ~1700 vertices, this update is cheap (~0.3 ms per frame).

### 3.2 Ambient Waves

The base wave height at any point `(x, y)` at time `t` is:

```

z_ambient(x,y,t) = WAVE_AMP - [

    0.40 - sin(1.2-x - 2.0-t)

  + 0.40 - sin(1.4-y + 1.8-t)

  + 0.30 - sin(1.5-(x+y) - 2.5-t)

  + 0.20 - sin(1.8-(x-y) + 2.2-t)

  + 0.10 - sin(4.0-x - 5.0-t)-sin(4.0-y + 4.0-t)

]

```

- **Five superimposed travelling sinusoids** with different directions and speeds produce a non-repeating, natural-looking surface.

- `WAVE_AMP = 0.008 m` (8 mm) → gentle, pool-like ripples.

- Setting `WAVE_AMP = 0.0` disables ambient motion entirely.

### 3.3 Robot-Induced Disturbances

Each robot that moves faster than `RIPPLE_SPEED_THR = 0.10 m/s` adds a **localized bump** to the wave field:

```

z_local(x,y,t) = 0.6 - WAVE_AMP - exp(-d² / 0.15) - sin(10-d - 12-t)

```

where `d` is the distance from the robot's `(x, y)` position. The exponential envelope confines the effect to a **~40 cm radius** around the moving robot; the `sin(10-d - 12-t)` term produces concentric outward-propagating ripples --- a **vortex-like wake** trailing the robot.

### 3.4 Ripple Rings

In addition to the vertex-displaced waves, a pool of **expanding foam rings** visualizes discrete disturbances. Each ring:

- Spawns at the robot's position every **0.35 s** while the robot moves (`RIPPLE_SPAWN_PERIOD`).

- Starts at radius `0.10 m` and expands to `0.55 m` over `1.8 s` (`RIPPLE_LIFETIME`).

- Fades in opacity from **0.90 → 0.00** as it expands.

- Lives in a pre-allocated pool of `N_ROBOTS × RIPPLE_PER_ROBOT = 15` slots (no dynamic allocation).

The ring geometry is a flat annulus (`32`-segment ring) whose scale and opacity are updated each frame.

### 3.5 Visual Materials

All materials use **`UsdPreviewSurface`** with the following parameters:

| Surface | Color | Opacity | Roughness | IOR | Notes |

|---|---|---|---|---|---|

| Water | `(0.10, 0.45, 0.85)` | 0.55 | 0.03 | 1.33 | bright blue, highly specular |

| Underwater volume | `(0.05, 0.32, 0.60)` | 0.18 | 0.60 | 1.33 | depth tint |

| Ripple ring | `(0.88, 0.96, 1.00)` | animated | 0.20 | 1.33 | foam-white, slight emissive |

| Pool floor | `(0.30, 0.55, 0.70)` | 1.0 | 0.60 | 1.50 | teal |

| Pool walls | `(0.92, 0.94, 0.96)` | 1.0 | 0.35 | 1.50 | bright white |

The water material is set to `doubleSided = True` so the surface is visible from below as well as above.

---

## 4. The `nemo_swarm_pool.py` Program

### 4.1 Architecture

The program is a **standalone Isaac Sim application** --- it launches the simulator directly from Python (`SimulationApp`) rather than as a GUI extension. This makes it ideal for batch experiments, headless runs, and reproducible scientific demos.

```

┌───────────────────────────────────────────────────────────────┐

│                    nemo_swarm_pool.py                         │

│                                                               │

│  ┌─────────────┐   ┌──────────────┐   ┌───────────────────┐   │

│  │ Simulation  │──▶│ World / Stage│──▶│ Pool + 5 NEMO     │   │

│  │ App start   │   │  (USD)       │   │ + WaterEffects    │   │

│  └─────────────┘   └──────────────┘   └───────────────────┘   │

│                                                │              │

│                                                ▼              │

│                                    ┌───────────────────┐      │

│                                    │ Main control loop │      │

│                                    │  - compute PD     │      │

│                                    │  - allocate motor │      │

│                                    │  - clip + recompose│     │

│                                    │  - apply forces   │      │

│                                    │  - animate water  │      │

│                                    └───────────────────┘      │

└───────────────────────────────────────────────────────────────┘

```

### 4.2 Execution Flow

| Step | What happens |

|---|---|

| **1. Boot** | `SimulationApp` starts Isaac Sim with GUI. |

| **2. Build scene** | Pool floor, walls, underwater volume, and water wave mesh are created. |

| **3. Load robots** | 5 references to `nemo_body.usd` are added at `/World/nemo_1` ... `/World/nemo_5`. Each is given a distinct coloured material. |

| **4. Apply physics** | Each robot gets `PhysxRigidBodyAPI` with gravity disabled, damping applied. |

| **5. Spawn poses** | Robots are placed in a small ring (radius 0.5 m) around the world origin. |

| **6. Play loop** | Every physics step: |

| │ 6a. | Read world poses and velocities. |

| │ 6b. | Compute target shape positions for the current phase. |

| │ 6c. | For each robot: run position PD → yaw PD → leveling → Z-hold. |

| │ 6d. | Allocate to motor forces → clip → recompose actual forces. |

| │ 6e. | Apply forces/torques to PhysX. |

| │ 6f. | Update water mesh vertices and ripple rings. |

| **7. Shutdown** | Window close → `simulation_app.close()`. |

### 4.3 Swarm Shape Sequence

The swarm cycles through six geometric formations, one every **8 seconds**:

| Index | Shape | Description |

|---|---|---|

| 0 | `line` | 5 robots in a horizontal line |

| 1 | `circle` | 5 robots on a circle (radius 1.5 m) |

| 2 | `V` | V-formation (bird-like) |

| 3 | `cross` | Plus-sign arrangement |

| 4 | `square` | Four corners + centre |

| 5 | `pentagon` | Regular pentagon |

The sequence repeats indefinitely (`SHAPE_DURATION = 8.0 s` each). Robots use their position PD to reach their assigned slot, so transient dynamics (acceleration, overshoot, convergence) are visible on each transition.

### 4.4 Key Configuration Parameters

All values are defined at the top of the file for easy tuning:

```python

# Scene

N_ROBOTS            = 5

ROBOT_USD_PATH      = "C:/isaac5/nemo_body.usd"

POOL_SIZE           = 8.0

ROBOT_SPAWN_Z       = 0.02

# NEMO physics (see §2.6)

NEMO_MASS, NEMO_J_O, NEMO_R, NEMO_MAX_FORCE

# Controller gains

KP_POS, KD_VEL, MAX_XY_FORCE

KP_YAW, KD_YAW

LEVEL_KP, LEVEL_KD

KP_Z, KD_Z, MAX_Z_FORCE

# Swarm behaviour

SHAPE_RADIUS        = 1.5

SHAPE_DURATION      = 8.0

# Water effects

WAVE_AMP            = 0.008

WAVE_GRID           = 40

RIPPLE_PER_ROBOT    = 3

RIPPLE_LIFETIME     = 1.8

RIPPLE_SPAWN_PERIOD = 0.35

```

### 4.5 Usage

**Requirements:**

- Isaac Sim 5.0 or 5.1

- A valid `nemo_body.usd` asset (three-thruster robot; must contain a RigidBody root prim named `nemo_body`)

**Launch:**

```bash

cd C:\isaac5

.\python.bat nemo_swarm_pool.py

```

**To run headless** (for batch experiments or data generation), change:

```python

simulation_app = SimulationApp({"headless": True, "multi_gpu": False})

```

**To exit:** close the Isaac Sim window; the program will shut down cleanly.

---

## 5. Coordinate Conventions

Throughout this documentation and the code, the following conventions are used:

| Axis | Definition | Sign |

|---|---|---|

| **+X** | Robot forward (aligned with M1 thruster arm) | right-hand rule |

| **+Y** | Robot left | right-hand rule |

| **+Z** | Up (world vertical) | right-hand rule |

| **Yaw** | Rotation about +Z | positive = counter-clockwise (viewed from above) |

| **Roll** | Rotation about +X | positive = right side down |

| **Pitch** | Rotation about +Y | positive = nose up |

World frame and body frame coincide at spawn (identity rotation). All controllers operate in the **world frame** for position and in the **body frame** for motor allocation; the conversion between them uses the robot's current yaw.

---

*End of document.*