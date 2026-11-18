# Multi-Agent C++ Migration – Ground Agent
**ROS2 Iron · C++ · Nav2 · Gazebo Fortress · pluginlib · GTest · Python (AI service)**

C++ version of the ground agent from my MSc multi-agent hazard detection system. It is not a direct migration: improvements have been added along the way. The ground agent responds to hazards in an open-cut mining terrain: it drives to the hazard, performs the on-arrival task, returns to base and parks. A web-based supervisory UI lets an operator choose a location to scan and approve or cancel each mission.

> **Status – Phase 1 (this repository state):** full mission flow for both hazards, **without orbiting**.
> **Phase 2 (work in progress):** adds the orbit task for the fibrous hazard.

---

## Demo

| Video | Shows |
|-------|-------|
| [Both hazards traversal with UI](media/BothHazardsTraversalWithUI.mp4) | Dust plume and fibrous hazard (initial inspection) missions, start to finish, driven from the web UI |
| [Alternate flows with UI](media/AlternateWithUI.mp4) | Alternate flows: override timeout with no approval or cancel, operator cancel, and no hazard found |

### Mining terrain

<p align="center">
  <img src="media/Mining%20Terrain%20Overall.png" alt="Overall view of the mining terrain used in the simulation" width="800"><br>
  <em>The mining terrain used in the simulation</em>
</p>

<p align="center">
  <img src="media/Mining%20Terrain%20-%20Hazards.png" alt="Mining terrain showing the dust plume and fibrous hazard locations" width="800"><br>
  <em>The hazard locations on the terrain</em>
</p>

<p align="center">
  <img src="media/Mining%20Terrain-In%20Mission.png" alt="The ground robot driving across the mining terrain during a mission" width="800"><br>
  <em>The ground robot during a mission</em>
</p>

---

## What the system does

> **Note:** the aerial agent from the MSc project has not been migrated. The supervisory agent and the hazard locations config are used as a proxy: the assumption is that an aerial agent has already detected the hazard and its location details are stored in a `.json` file.

1. The operator selects a location to scan in the supervisory UI. Depending on the configuration for that location, the scan either reports **hazard detected** or **no hazard found**. When a hazard is detected, a `HazardPose` message (location, type, estimated diameter) is published and the ground agent picks it up.
2. The supervisory UI shows the alert. The operator **approves** or **cancels**; if nobody responds before the configured timeout, the mission proceeds.
3. The ground agent plans a path with Nav2 and drives to the hazard.
4. On arrival it performs the hazard-specific task:
   - **Dust plume** – relays dust sensor readings while an AI model estimates plume density from the camera image, in parallel. These are two separate results; the dust plume task reports back only once both have been published.
   - **Fibrous hazard** – spins in place and captures a set of images (initial inspection). Orbiting follows in Phase 2.
5. The agent returns to base and parks at the configured heading, ready for the next mission.

---

## Architecture

| Component | Type | Responsibility |
|-----------|------|----------------|
| `MissionHandlerNode` | Node | Orchestrates the mission: override window, path request, navigation, on-arrival hand-off, return, parking |
| `Nav2PathPlannerPlugin` | Plugin | Computes a path with Nav2, down-samples it to waypoints, sets orientations |
| `Nav2NavigationPlugin` | Plugin | Drives the waypoints (NavigateThroughPoses) and performs the parking rotation (Spin) |
| `OnArrivalTaskNode` | Node | Central point for all hazard tasks: forwards the task to the right plugin or node, listens for completion and reports back to the mission handler |
| `InitialInspectionPlugin` | Plugin | Spins in place and saves images at fixed angle steps |
| `DustPlumeActionsNode` | Node | Runs dust sensing and AI density estimation in parallel; reports back only once both results have been published |
| `DustSensorRelayPlugin` | Plugin | Relays dust sensor readings above a threshold |
| `DustPlumeEstimationPlugin` | Plugin | Sends a camera image to the Python AI service and derives a severity level |
| `DensityRegressorNode` | Python node | AI service (ResNet-18 density regressor) |
| Supervisory agent | Nodes + web UI | Location scan, alerts, approve/cancel override, mission status |

Nodes load their plugins through **pluginlib** against base interfaces, so a plugin can be replaced without touching the node.

Details: [MissionHandlerNode](docs/mission_handler_node.md) · [OnArrivalTaskNode](docs/on_arrival_task_node.md) · [DustPlumeActionsNode](docs/dust_plume_actions_node.md) · [Shared infrastructure](docs/shared_infra.md) · [Supervisory agent](docs/supervisory_agent.md)

<details>
<summary><b>Navigation and localisation</b></summary>

- Nav2 provides path planning (`ComputePathToPose`), waypoint following (`NavigateThroughPoses`) and the parking rotation (`Spin`).
- Localisation fuses odometry and IMU with a UKF; TF is used for the robot pose.
- Nav2 parameters were tuned for the terrain and are a large part of the speed-up over the MSc version (see Performance).

</details>

<details>
<summary><b>Dust plume: parallel sensing and AI estimation</b></summary>

- `DustPlumeActionsNode` starts the sensor relay and the density estimation at the same time.
- The estimation plugin waits (with timeout) for a camera image handed over from the image callback using a mutex and condition variable, then calls the Python `density_regressor` service.
- The sensor readings and the density estimate are two separate results. The node reports back only once both have been published; a timeout or failure in either is reported as a failed mission.
- All AI stays in Python; the C++ side only calls the service.

</details>

<details>
<summary><b>Simulation</b></summary>

- Gazebo Fortress, outdoor mining terrain. The terrain is different from the one used in the MSc project, and about twice the size.
- The dust plume is a hazard overlay in SDF with a custom C++ Gazebo plugin (`DustPlumeSensorPlugin`) that provides dust readings.
- Robot: Agilex (see Third-party assets).

</details>

<details>
<summary><b>Testing</b></summary>

All code written for this project is unit and integration tested for real, with one exception: the dust sensor Gazebo plugin only works inside Gazebo, so in unit and integration tests it is replaced by a stand-in publisher. Nav2, TF, odometry, camera and velocity topics are also faked as test inputs.

- **Plugin tests** – each plugin tested individually.
- **Node tests** – each node tested with its real plugins.
- The AI density service is not faked; the tests launch it with Boost.Process.
- - The supervisory agent & web UI has no unit tests: it is not a core part of the project. It is there only as a proxy for the aerial agent, and for displaying status and overriding missions.
- **End-to-end tests** – no fakes were used. A single application start runs two missions from the web UI, one after the other: four legs in total (two to the destination, two back to base). The test can also run just one hazard, or both in any order.

</details>

<details>
<summary><b>Shared infrastructure and transport interfaces</b></summary>

- `AgentUtils` – JSON config access, TF lookup, conversions, geometry helpers, bag recording and many other utility methods.
- `transport_interfaces` – ROS2 messages, services and actions. `RequestPathPlan` is an action.
- See [Shared infrastructure](docs/shared-infrastructure.md).

</details>

---

## Folder structure

<details>
<summary><b>Show structure</b></summary>

```
├───code
│   └───ros2_ws
│       └───src
│           ├───ai_python
│           │   ├───ai_models
│           │   ├───ai_python
│           │   │   └───nodes
│           │   ├───resource
│           │   └───test
│           ├───ground_agent_dust_plume_actions_plugins
│           │   ├───include
│           │   │   ├───dust_plume_estimation_plugin
│           │   │   └───dust_sensor_relay_plugin
│           │   ├───source_files
│           │   └───test
│           ├───ground_agent_mission_handler_plugins
│           │   ├───include
│           │   │   ├───navigation_plugin
│           │   │   └───path_planner_plugin
│           │   ├───source_files
│           │   └───test
│           ├───ground_agent_nodes
│           │   ├───config
│           │   ├───include
│           │   ├───launch
│           │   ├───source_files
│           │   └───test
│           ├───ground_agent_on_arrival_task_plugins
│           │   ├───include
│           │   │   ├───initial_inspection_plugin
│           │   │   └───orbit_hazard_plugin
│           │   ├───source_files
│           │   └───test
│           ├───multi_agent_simulation
│           │   ├───config
│           │   │   └───nav2_bt_files
│           │   ├───include
│           │   ├───launch
│           │   ├───maps
│           │   ├───models
│           │   │   ├───agilex
│           │   │   │   └───meshes
│           │   │   ├───ground_plane
│           │   │   ├───hazard_overlays
│           │   │   │   ├───asbestos_marker
│           │   │   │   │   └───materials
│           │   │   │   │       └───textures
│           │   │   │   └───dust_plume_marker
│           │   │   │       └───materials
│           │   │   │           └───textures
│           │   │   │               └───generated_textures
│           │   │   ├───mining_terrain
│           │   │   │   ├───materials
│           │   │   │   │   ├───scripts
│           │   │   │   │   └───textures
│           │   │   │   ├───meshes
│           │   │   │   └───thumbnails
│           │   │   ├───obstacles
│           │   │   │   ├───falling_rock1
│           │   │   │   │   ├───materials
│           │   │   │   │   │   ├───scripts
│           │   │   │   │   │   └───textures
│           │   │   │   │   ├───meshes
│           │   │   │   │   └───thumbnails
│           │   │   │   ├───medium_rock_fall
│           │   │   │   │   ├───materials
│           │   │   │   │   │   ├───scripts
│           │   │   │   │   │   └───textures
│           │   │   │   │   ├───meshes
│           │   │   │   │   └───thumbnails
│           │   │   │   ├───mossy_bark
│           │   │   │   │   ├───materials
│           │   │   │   │   │   ├───scripts
│           │   │   │   │   │   └───textures
│           │   │   │   │   └───thumbnails
│           │   │   │   ├───small_rock_fall
│           │   │   │   │   ├───materials
│           │   │   │   │   │   ├───scripts
│           │   │   │   │   │   └───textures
│           │   │   │   │   ├───meshes
│           │   │   │   │   └───thumbnails
│           │   │   │   └───wood_case
│           │   │   │       ├───materials
│           │   │   │       │   ├───scripts
│           │   │   │       │   └───textures
│           │   │   │       ├───meshes
│           │   │   │       └───thumbnails
│           │   │   ├───office_building
│           │   │   │   ├───meshes
│           │   │   │   └───thumbnails
│           │   │   └───wall
│           │   ├───multi_agent_simulation
│           │   ├───resource
│           │   ├───rviz
│           │   ├───scripts
│           │   ├───source_files
│           │   ├───urdf
│           │   ├───visualisation
│           │   └───worlds
│           ├───shared_infrastructure
│           │   ├───config
│           │   ├───images
│           │   │   ├───initial_inspection
│           │   │   └───test
│           │   ├───include
│           │   │   ├───dust_plume_sensor_plugin
│           │   │   ├───orbit_controller_plugin
│           │   │   ├───shared_infrastructure
│           │   │   └───test_helpers
│           │   │       └───nav2
│           │   └───source_files
│           │       └───test_helpers
│           ├───supervisory_agent
│           │   ├───config
│           │   ├───include
│           │   ├───launch
│           │   ├───source_files
│           │   └───web_ui
│           └───transport_interfaces
│               ├───action
│               ├───msg
│               └───srv
|───docs
└───media
```

</details>

---

## Configuration

| File | Purpose |
|------|---------|
| `config.json` (in `shared_infrastructure/config`) | Topic names, timeouts, override timeout, parking heading, dust relay,  image settings, and all other settings |
| `hazard_locations.json` | Hazard locations used by the simulation, and whether a location reports a hazard or none (stands in for the aerial agent) |

---

## How to run

<<how_to_run_commands>>

---

## Performance

Measured in simulation, return to base included:

- **C++ (this project):** about 3 minutes for both hazard missions together (one video).
- **MSc Python version (ground agent):** dust plume about 8 minutes; fibrous hazard about 4.5 minutes to the end of the initial inspection (about 12.5 minutes for both).

The improvement comes from the C++ version together with better Nav2 configuration. The terrain also differs from the MSc one (and is about twice the size), which may have had an impact. A full fibrous-mission comparison, including orbiting, follows with Phase 2.

---

## Third-party assets

- Agilex robot description based on `scout_ros2` (Apache 2.0). Meshes are unmodified; the URDF is modified. See `NOTICE` and `LICENSE`.

---

## Motivation

- Lower latency and better determinism than the Python agents
- Production-style structure: replaceable plugins, tested components
- Hands-on depth in Nav2 configuration and ROS2 concurrency

## Related projects

- MSc system (Python): [multi-agent-hazard-detection](https://github.com/somikm-robotics/multi-agent-hazard-detection)


## Tech stack

| Category | Technologies |
|----------|-------------|
| Framework | ROS2 Iron, Nav2, pluginlib |
| Languages | C++, Python (AI service), JavaScript (UI) |
| Simulation | Gazebo Fortress |
| Localisation | UKF + IMU, TF |
| AI | ResNet-18 density regressor |
| Build | CMake, colcon |
| Testing | GTest, Boost.Process |
| UI | HTTP server with SSE, web dashboard |
