# Nav2 Sim-to-Real – Autonomous Navigation & Orbit Inspection
**ROS2 · Nav2 · Gazebo · TurtleBot4 Lite · C++ · SLAM Toolbox**

Autonomous navigation extending the MSc multi-agent hazard detection system from simulation to real hardware deployment. Covers mapping, localisation, path planning, and a custom C++ Nav2 orbit controller for hazard inspection — validated in both Gazebo simulation (Agilex) and real-world deployment (TurtleBot4 Lite).

---

## Overview

| Package | Platform | Environment |
|---------|----------|-------------|
| `indoor_ground_agent` | Agilex wheeled robot | Gazebo simulation (indoor world) |
| `turtlebot` | TurtleBot4 Lite | Real hardware (indoor) |

Both packages share the same core navigation and orbit inspection architecture. The TurtleBot4 Lite deployment validated the simulation-based work on physical hardware, surfacing and resolving real-world deployment challenges.

---

## Key Engineering Work

### Autonomous Navigation
- SLAM-based mapping using SLAM Toolbox on both simulated and real environments
- Localisation and path planning via Nav2
- Custom Nav2 behaviour trees for navigation control (`config/bt/`)
- Maps generated and validated on both platforms (`maps/`)

### Custom C++ Nav2 Orbit Controller
A custom Nav2 controller plugin (`orbit_controller.cpp`) enabling autonomous orbiting of a target at a consistent radius while maintaining sensor alignment:
- Implements Nav2 controller plugin interface in C++
- Integrates LaserScan data for obstacle-aware behaviour during orbit
- Directly extends the orbital survey behaviour from the MSc fibrous hazard mission
- Adapted independently for both Agilex (simulation) and TurtleBot4 Lite (real hardware)

### Real-World Deployment Challenges (TurtleBot4 Lite)
Deploying on physical hardware exposed several platform-specific constraints:
- **CPU limitations** — TurtleBot4 Lite Raspberry Pi compute constraints required system-level tuning
- **LaserScan timing** — `scan_fresh_relay_node.py` developed to resolve stale scan data issues affecting navigation reliability
- **Orbit twist corrections** — `orbit_twist_corrected_node.py` developed to address TB4 Lite-specific motion control issues during orbital manoeuvres
- **TF inconsistencies** — frame alignment issues between simulation and real hardware resolved through configuration tuning

---

## Package Structure

### indoor_ground_agent (Agilex — Simulation)
```
indoor_ground_agent/
├── config/
│   ├── nav2_params.yaml
│   ├── slam_toolbox_params.yaml
│   └── bt/                          ← custom behaviour trees
├── include/orbit_controller/
│   └── orbit_controller.hpp         ← C++ plugin header
├── plugins/
│   └── orbit_controller.cpp         ← C++ Nav2 controller plugin
├── indoor_ground_agent/nodes/
│   ├── orbit_generator.py
│   ├── orbit_manager.py
│   └── twist_relay_node.py
├── launch/
├── maps/                            ← SLAM-generated indoor map
├── worlds/                          ← Gazebo indoor world
└── visualisations/                  ← RViz config
```

### turtlebot (TurtleBot4 Lite — Real Hardware)
```
turtlebot/
├── config/
│   ├── nav2_params.yaml
│   ├── localisation_params.yaml
│   └── bt/                          ← custom behaviour trees
├── include/orbit_controller/
│   └── orbit_controller.hpp         ← C++ plugin header
├── plugins/
│   └── orbit_controller.cpp         ← C++ Nav2 controller plugin
├── turtlebot/nodes/
│   ├── orbit_manager_node.py
│   ├── orbit_twist_node.py
│   ├── orbit_twist_corrected_node.py ← TB4 Lite motion correction
│   └── scan_fresh_relay_node.py      ← LaserScan timing fix
├── launch/
└── maps/                            ← TB4 Lite real-world map
```

---

## Third-Party Models

Agilex robot meshes are not included in this repository due to file size.
Originally downloaded from [Agilex Robotics](https://github.com/agilexrobotics)
and modified for this project. Modified versions are available in the 
parent project:

| Asset | Location |
|-------|----------|
| Agilex meshes (modified) | [multi-agent-hazard-detection](https://github.com/somikm-robotics/multi-agent-hazard-detection) — `ground_agent/models/` |

Copy into `indoor_ground_agent/models/agilex/` before running simulation.

---

## Tech Stack

| Category | Technologies |
|----------|-------------|
| Robotics Framework | ROS2, Nav2 |
| Simulation | Gazebo Fortress |
| Hardware | TurtleBot4 Lite, Raspberry Pi |
| Languages | Python · C++ (orbit controller plugin) |
| Navigation | SLAM Toolbox, Nav2 behaviour trees |
| Sensing | LiDAR, LaserScan |
