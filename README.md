# Motion Planning and Decision Making for Autonomous Vehicles
## Udacity Self-Driving Car Engineer Nanodegree — Course 5 Project

---

## Project Overview

This project implements a hierarchical motion planning system for an autonomous vehicle operating in the CARLA simulator. The planner receives perception data from CARLA via a Python bridge (`simulatorAPI.py`), computes optimal trajectories using a behavior FSM and cubic spiral paths, and returns velocity-profiled trajectories for the vehicle controller.

The system demonstrates:
- **Behavioral decision making** via a Finite State Machine (FSM)
- **Spiral path generation** using Newton-Raphson optimization
- **Collision checking** using circle-based footprint approximation
- **Velocity profiling** with trapezoidal acceleration/deceleration

---

## Architecture

```
CARLA Simulator (Docker, port 2000)
        |
        | (Python API — simulatorAPI.py connects CARLA + planner)
        |
simulatorAPI.py  ←──── WebSocket (port 4567) ────→  C++ Planner (./planner)
        |
        |  JSON: {ego_x, ego_y, yaw, velocity, waypoint, obstacles, ...}
        ↓
┌─────────────────────────────────────────────────────┐
│                   C++ Planner (main.cpp)             │
│                                                      │
│  ┌──────────────────────┐                            │
│  │  Behavior Planner FSM│                            │
│  │  (behavior_planner_  │  → Goal State (x,y,θ,v)   │
│  │   FSM.cpp)           │                            │
│  └──────────┬───────────┘                            │
│             ↓                                        │
│  ┌──────────────────────┐                            │
│  │   Motion Planner     │  → Offset Goals (7 paths)  │
│  │  (motion_planner.cpp)│                            │
│  └──────────┬───────────┘                            │
│             ↓                                        │
│  ┌──────────────────────┐                            │
│  │  Cubic Spiral + NR   │  → 7 Candidate Spirals     │
│  │  (cubic_spiral.cpp)  │                            │
│  └──────────┬───────────┘                            │
│             ↓                                        │
│  ┌──────────────────────┐                            │
│  │   Cost Functions     │  → Best Trajectory Index   │
│  │ (cost_functions.cpp) │                            │
│  └──────────┬───────────┘                            │
│             ↓                                        │
│  ┌──────────────────────┐                            │
│  │  Velocity Profile    │  → (x, y, v) trajectory    │
│  │    Generator         │                            │
│  └──────────────────────┘                            │
└─────────────────────────────────────────────────────┘
        |
        | JSON: {trajectory_x, trajectory_y, trajectory_v, spirals, ...}
        ↓
CARLA vehicle controller
```

---

## Key Algorithms

### 1. Behavior FSM
States: `FOLLOW_LANE`, `DECEL_TO_STOP`, `STOPPED`, `LANE_CHANGE_LEFT`, `LANE_CHANGE_RIGHT`

Transitions:
- `FOLLOW_LANE` → `DECEL_TO_STOP` when a junction/stop line is detected within lookahead distance
- `DECEL_TO_STOP` → `STOPPED` when ego is within `P_STOP_THRESHOLD_DISTANCE` of the stop line
- `STOPPED` → `FOLLOW_LANE` after `P_REQ_STOPPED_TIME` seconds have elapsed

### 2. Cubic Spiral Path Generation
Curvature polynomial: κ(s) = a + b·s + c·s² + d·s³

Integration via Frenet equations:
```
dx/ds = cos(θ(s))
dy/ds = sin(θ(s))
dθ/ds = κ(s)
```

Newton-Raphson optimizer finds spiral parameters so the endpoint matches the desired goal state. The Jacobian is computed via Simpson's rule numerical integration.

### 3. Circle-Based Collision Checking
Each vehicle (ego and obstacles) is approximated as 3 circles placed along the vehicle body. A path is in collision if any ego circle overlaps any obstacle circle:
```
collision = (dist < CIRCLE_RADII[c] + CIRCLE_RADII[c2])
```

### 4. Trapezoidal Velocity Profiling
Physics-based speed calculation:
- Acceleration phase: `v_f² = v_i² + 2·a·d`
- Distance for speed change: `d = (v_f² - v_i²) / (2·a)`

---

## Parameters (`planning_params.h`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `P_NUM_PATHS` | 7 | Lateral offset paths generated |
| `P_GOAL_OFFSET` | 1.5 m | Spacing between adjacent path goals |
| `P_ERR_TOLERANCE` | 1e-4 | Newton-Raphson convergence threshold |
| `P_NUM_POINTS_IN_SPIRAL` | 20 | Path points per spiral |
| `P_LOOKAHEAD_MIN` | 8.0 m | Minimum lookahead distance |
| `P_LOOKAHEAD_MAX` | 20.0 m | Maximum lookahead distance |
| `P_LOOKAHEAD_TIME` | 1.5 s | Lookahead time multiplier |
| `P_SPEED_LIMIT` | 3.0 m/s | Target cruise speed |
| `P_STOP_THRESHOLD_SPEED` | 0.02 m/s | Speed below which ego is "stopped" |
| `P_REQ_STOPPED_TIME` | 1.0 s | Duration to hold stop before resuming |
| `P_REACTION_TIME` | 0.25 s | Safety reaction time |
| `P_MAX_ACCEL` | 1.5 m/s² | Maximum acceleration |
| `P_SLOW_SPEED` | 1.0 m/s | Creep speed before braking |
| `P_STOP_LINE_BUFFER` | 0.5 m | Distance before stop line to halt |
| `P_LANE_WIDTH` | 4.0 m | Lane width |
| `P_DIST_COLLISION_THRESH` | 4.5 m | Collision circle threshold |
| `P_CIRCLE_DIST` | 1.5 m | Spacing between vehicle circles |
| `P_NUM_CIRCLES` | 3 | Vehicle footprint circles |
| `P_LANE_CHANGE_COOLDOWN_TICKS` | 150 | Ticks to block next lane change (~3s) |

---

## Setup and Run Instructions

### Prerequisites
- Ubuntu 20.04
- NVIDIA GPU with driver ≥ 450 and nvidia-docker2 installed
- Docker with NVIDIA runtime
- cmake ≥ 3.10, g++ ≥ 7, make
- CARLA client C++ libraries at `/opt/carla-source` (see Build Notes)

### Step 1: Start CARLA (Docker)
```bash
# Add yourself to the docker group first (one-time):
sudo usermod -aG docker $USER && newgrp docker

# Start CARLA server:
bash scripts/start_carla.sh
```

### Step 2: Build the Planner
```bash
# First time: install CARLA client libraries
bash scripts/setup_build_env.sh   # takes 10-30 min, one-time

# Then build:
bash scripts/build_only.sh
```

### Step 3: Run the Full Simulation
```bash
bash scripts/build_and_run.sh
```

### Step 4: Stop Everything
```bash
bash scripts/stop_carla.sh
```

### Build Notes
The C++ planner links against the CARLA client library (`libcarla_client.a`) and Boost.
These are available in:
1. **Udacity workspace** — already pre-installed at `/opt/carla-source`
2. **Local setup** — run `bash scripts/setup_build_env.sh` to download and compile (requires ~10GB disk space)

---

## Improvements Over Baseline

1. **Lane Change Hysteresis** (`behavior_planner_FSM.cpp`): Added `_lane_change_cooldown` counter that blocks rapid consecutive lane changes for 3 seconds (150 ticks at 20ms).

2. **Configurable Cost Weights** (`cost_functions.h`): Added `CostWeights` struct with tunable `collision_weight`, `proximity_weight`, and `lane_center_weight` fields.

3. **Path Validity Logging** (`motion_planner.cpp`): Each planning tick prints per-path status: `[MotionPlanner] Path 3/7: cost=0.234 VALID` or `COLLISION`.

4. **Emergency Fallback Path** (`motion_planner.cpp`): If all 7 paths collide, selects the path with maximum minimum-distance to obstacles instead of returning an empty result.

5. **Velocity Smoothing** (`velocity_profile_generator.cpp`): Applied 3-point moving average to velocity profiles to reduce jerk at speed transitions.

---



## Author

**Shyam Shenoi**
