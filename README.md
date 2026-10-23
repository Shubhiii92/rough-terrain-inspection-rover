# Four-Wheel Rough-Terrain Inspection Rover (Webots R2025a & 3D Web Simulation)

A complete, demonstrable four-wheel Rough-Terrain Inspection Rover project featuring:
1. **Interactive 3D Web Simulation**: Live in-browser simulation with real-time ground collision, 4-wheel independent suspension terrain-following, obstacle avoidance FSM, live telemetry dashboard, and 7 test scenarios.
2. **Webots R2025a Project**: Native Webots simulation with parameterized `RoughTerrainRover.proto`, calibrated piecewise distance sensors, and autonomous Python controller `rough_terrain_controller.py`.

🌐 **Live Deployment on GitHub Pages:**  
👉 **[https://shubhiii92.github.io/rough-terrain-inspection-rover/](https://shubhiii92.github.io/rough-terrain-inspection-rover/)**

---

## 1. Project Overview & Architecture

The **Rough-Terrain Inspection Rover** is engineered to navigate cluttered outdoor environments, evaluate lateral clearances, negotiate rocky berms, and climb inspection ramps.

```text
rough-terrain-inspection-rover/
├── index.html                           # Full 3D Web simulation & interactive telemetry dashboard
├── worlds/
│   └── rough_terrain_rover.wbt          # Native Webots R2025a world file
├── controllers/
│   └── rough_terrain_controller/
│       └── rough_terrain_controller.py  # Python autonomous FSM controller
├── protos/
│   └── RoughTerrainRover.proto          # Parameterized 4WD rover robot definition
├── docs/
│   └── simulation_test_plan.md          # Verification test procedures & criteria
├── .github/workflows/
│   └── deploy-pages.yml                 # Automated GitHub Pages CI/CD deployment
├── RoughTerrainRover_Webots_R2025a.zip  # 1-click downloadable archive
└── README.md                            # Comprehensive project guide & documentation
```

---

## 2. Terrain Sinking Bug: Root Cause & Architectural Fix

### Root Cause Analysis
- **The Issue:** The rover previously sank 15–25 cm below the ground or floated in mid-air when travelling across uneven terrain.
- **Root Cause:** 
  1. The procedural terrain mesh applied height displacements $Y = f(x, z)$ across the landscape, but the rover kinematics loop only updated horizontal positions ($X$ and $Z$). 
  2. The chassis elevation ($Y$) was hardcoded to a fixed height ($Y = 0$), causing the ground to rise above the chassis whenever driving over positive elevation features.
  3. Rover pitch and roll were hardcoded heuristics based on heading sine waves, rather than physical wheel ground contact points.

### Implemented Fix
1. **Exact Mathematical Heightfield (`getTerrainElevation` & `getTerrainNormal`):**
   - Implemented an analytical multi-octave height function with continuous partial derivatives for terrain elevation and surface normals anywhere on the map.
2. **4-Wheel Ground Contact & Attitude Solver (`updateTerrainContactPhysics`):**
   - Transformed each wheel's offset into world coordinates to sample individual contact elevations: $y_{FL}, y_{FR}, y_{RL}, y_{RR}$.
   - Calculated true chassis elevation from the average wheel contact plane:  
     $$y_{\text{chassis}} = \frac{y_{FL} + y_{FR} + y_{RL} + y_{RR}}{4} + R_{\text{wheel}}$$
   - Computed physical longitudinal pitch from front vs. rear axle elevation:  
     $$\theta_{\text{pitch}} = \arctan\left(\frac{y_{\text{front}} - y_{\text{rear}}}{\text{Wheelbase}}\right)$$
   - Computed physical lateral roll from right vs. left axle elevation:  
     $$\theta_{\text{roll}} = \arctan\left(\frac{y_{\text{right}} - y_{\text{left}}}{\text{TrackWidth}}\right)$$
   - Applied smooth Quaternion orientation (`qHeading * qPitch * qRoll`), eliminating Euler gimbal lock and jitter.
3. **Suspension Travel Articulation:**
   - Each wheel articulates along its vertical suspension arm within physical spring limits ($\pm 6\,\text{cm}$), keeping all four wheels glued to the terrain surface.
4. **Solid Boulder Collision & Slide Response:**
   - Added collision sphere repulsion and normal velocity dampening to prevent clipping through rocks.

---

## 3. Autonomous Navigation & FSM Controller

The autonomous controller implements a calibrated Finite-State Machine:

```
[FORWARD_CRUISE] ──(Obstacle < 0.65m)──> [SLOW_APPROACH]
       │                                       │
       │                                (Obstacle < 0.35m)
       │                                       ↓
       │                              [AVOIDANCE_TURN]
       │                                       │
 (Obstacle < 0.18m)                      (Clearance > 0.60m)
       ↓                                       ↓
[EMERGENCY_REVERSE] ──(Timer Expired)──> [FORWARD_CRUISE]
       │
 (Stuck > 1.5s)
       ↓
[STUCK_RECOVERY] ────(Multi-Point)─────> [FORWARD_CRUISE]
```

- **Sensor Suite:** 3 Calibrated Infrared Distance Sensors (Front-Left $+28^\circ$, Front-Center $0^\circ$, Front-Right $-28^\circ$).
- **Anti-Oscillation Memory:** When an obstacle is detected, the controller commits to a turn direction based on lateral clearance comparison until the path is cleared, preventing rapid left-right switching.
- **Anti-Stuck Recovery:** Monitors displacement over 1.5s windows. If linear speed is commanded but displacement $< 0.08\,\text{m}$, initiates an automatic multi-step recovery sequence.

---

## 4. Repeatable Test Scenarios

The 3D Web simulation includes 7 built-in test scenarios accessible via the top dropdown:

| Scenario | Objective | Validation Criteria |
|---|---|---|
| **Scenario 1: Open Terrain Traverse** | Free exploration over rolling earth hills | Ground contact maintained; no sinking or floating |
| **Scenario 2: Single Boulder Avoidance** | Direct forward approach toward large rock | Smooth slowdown at 0.65m, pivot turn at 0.35m, path clear |
| **Scenario 3: Rocky Slalom Maze** | Traversing dense alternating boulder field | Alternating avoidance without getting trapped |
| **Scenario 4: Narrow Canyon Gate** | Navigating between two closely-spaced boulders | Evaluates width clearance; turns away if gap $< 0.8\text{m}$ |
| **Scenario 5: Steep Ridge & Berm** | Climbing elevated northwest hill and ramp | Chassis pitch reflects slope; suspension stays grounded |
| **Scenario 6: Dead-End U-Trap Escape** | Spawning inside a three-sided rock barrier | Triggers reverse backoff and multi-point exit turn |
| **Scenario 7: Autonomous Waypoint Patrol** | Navigating a 4-station inspection circuit | Reaches beacons Alpha, Bravo, Charlie, Delta sequentially |

---

## 5. Controls & Camera Modes

### Drive Modes
- **Autonomous AI:** Fully autonomous obstacle avoidance and terrain negotiation.
- **Manual Teleoperation:** Direct user control using `W`/`Up` (Forward), `S`/`Down` (Reverse), `A`/`Left` (Turn Left), `D`/`Right` (Turn Right), and `Spacebar` (Emergency Brake).

### Camera Modes
- **Follow Cam:** Smooth camera chase following behind the rover with ground clearance dampening.
- **Orbit 360°:** Free mouse rotation, pan, and zoom.
- **Mast FPV:** First-person view directly from the rover's mast inspection camera with HUD reticle.
- **Top Down:** Overhead tactical map overview.

---

## 6. Running Locally

### Browser 3D Simulation
```bash
cd rover
python3 -m http.server 8000
# Open http://localhost:8000 in your browser
```

### Native Webots Simulator
1. Launch **Webots R2025a**.
2. Select **File &rarr; Open World** and choose `worlds/rough_terrain_rover.wbt`.
3. Press **Run** to execute the simulation and Python controller.
