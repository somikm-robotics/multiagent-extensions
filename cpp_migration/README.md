# Multi-Agent C++ Migration
**ROS2 · C++ · Nav2 · Gazebo**

Migration of Python-based ROS2 agents from the MSc multi-agent hazard detection system to C++, targeting production-grade performance and real-world deployment readiness.

---

## Overview

The MSc multi-agent system was implemented in Python within a ROS2 simulation environment. This sub-project migrates the core agent components to C++ to improve real-time performance, reduce latency, and prepare the system for deployment on resource-constrained hardware.

The migration uses a **plugin-based architecture** — each functional component is implemented as a replaceable C++ plugin, maintaining the same ROS2 communication interfaces as the Python system. The entire architecture is designed to be thread-safe, supporting concurrent plugin execution without race conditions.

> **Note:** Source code will be added as implementation progresses.
> Current repository contains architecture, headers, and interface
> definitions only.

---

## Architecture

### Ground Agent Packages

**`ground_agent_nodes`** — core ROS2 nodes
- `MissionHandlerNode` — central orchestrator
- `ArrivalTaskNode` — on-arrival task coordination
- `DustPlumeDensityEstimationNode` — real-time density estimation

**`ground_agent_mission_handler_plugins`** — navigation and path planning plugins
- `NavigationPlugin` / `Nav2NavigationPlugin` — Nav2-based navigation execution
- `PathPlannerPluginBase` / `Nav2PathPlannerPlugin` — Nav2 path planning
- `AStarPathPlannerPlugin` — A* path planning alternative

**`ground_agent_on_arrival_task_plugins`** — on-arrival behaviour plugins
- `InitialInspectionPlugin` — initial survey and image capture
- `OrbitHazardPlugin` / `Nav2CommanderOrbitHazardPlugin` / `TwistCommanderOrbitHazardPlugin` — orbital survey implementations
- `DustSensorRelayPlugin` — dust sensor data relay

**`ground_agent_dust_plume_density_estimation_plugins`** — AI-based density estimation
- Multi-threaded plugin invoking AI inference pipeline for real-time dust plume density estimation
- Implementation in progress

**`transport_interfaces`** — shared ROS2 message and service definitions (complete)

### Supervisory Agent
- Planned — implementation pending

---

## Transport Interfaces

Complete ROS2 message and service definitions matching the Python system:

**Messages:** `HazardPose` · `MissionStatus` · `MissionCompleted` · `NavigationResult` · `OnArrivalTask` · `BaseReturn` · `DustSensorResult` · `DensityEstimationResult` · `UIOverrideMission`

**Services:** `RequestPathPlan` · `PathPlanResult` · `NavigateRequest` · `OrbitHazard` · `InitialInspection` · `PlumeDensity` · `OverrideMission`

---

## Current Status

| Component | Status |
|-----------|--------|
| Plugin architecture and headers | ✅ Complete |
| Transport interfaces | ✅ Complete |
| Ground agent node implementation | 🔄 In progress |
| Ground agent plugin implementation | 🔄 In progress |
| Dust plume density estimation plugin | 🔄 In progress |
| Supervisory agent | ⏳ Planned |
| Simulation environment | ⏳ Planned |

---

## Motivation

- **Real-time performance** — C++ ROS2 nodes have lower latency and better determinism than Python equivalents
- **Production readiness** — C++ is the standard for production robotics deployment
- **Hardware constraints** — embedded platforms benefit significantly from C++ efficiency
- **Plugin architecture** — replaceable components for navigation, path planning, and inspection behaviours
- **Thread safety** — concurrent plugin execution designed for reliable real-world deployment

---

## Related Projects

- **MSc System (Python):** [multi-agent-hazard-detection](https://github.com/yourhandle/multi-agent-hazard-detection)
- **Warehouse Project (C++):** [warehouse-inspection-ros2](https://github.com/yourhandle/warehouse-inspection-ros2)

---

## Tech Stack

| Category | Technologies |
|----------|-------------|
| Robotics Framework | ROS2, Nav2 |
| Language | C++ |
| Simulation | Gazebo Fortress |
| Build System | CMake, colcon |
