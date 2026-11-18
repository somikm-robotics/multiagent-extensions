# Shared infrastructure

[← Back to README](../README.md)

Code and data used by more than one package: the utility class, shared types, configuration, the Gazebo dust sensor plugin and the test helpers. The ROS2 message, service and action definitions are in the separate `transport_interfaces` package, described at the end of this page.

## Contents of the `shared_infrastructure` package

| Folder | Contents |
|--------|----------|
| `config` | `config.json` – all agent settings |
| `include/shared_infrastructure` | `AgentUtils`, shared enums and data types |
| `include/dust_plume_sensor_plugin` | `DustPlumeSensorPlugin` – Gazebo plugin that provides dust readings |
| `include/orbit_controller_plugin` | Nav2 orbit controller plugin (Phase 2) |
| `include/test_helpers` | Fakes and helpers used by the tests (including Nav2 test helpers) |
| `images` | `initial_inspection` – output folder for captured images; `test` – input images used by the tests |

## AgentUtils

A static utility class used by the nodes and plugins. It provides JSON config access, TF lookup, conversions, geometry helpers, bag recording and many other utility methods.

<details>
<summary><b>Configuration access</b></summary>

Reads `config.json` once and exposes typed getters, for example:

- service timeout and navigation override timeout
- image output folder
- topic names (camera, velocity, dust sensor relay topic)
- parking target heading and Nav2 spin time allowance
- dust sensor relay parameters (maximum readings, threshold, minimum values available)
- base (Agilex) pose and TF frame settings
- orbit settings (Phase 2) and which orbit plugin to use

</details>

<details>
<summary><b>TF and geometry</b></summary>

- Transform lookup between the configured map and base frames, returning nothing if it is not available yet.
- Yaw / quaternion conversion, angle normalisation, yaw from pose.
- Distance between two poses.
- Building a pose from a simple pose (x, y, z, yaw).

</details>

<details>
<summary><b>Type conversions</b></summary>

String to enum and enum to string for hazard type, task type, severity level, command type and orbit mode.

</details>

<details>
<summary><b>Message and path helpers</b></summary>

- `getMessage(...)` builds a message from any number of arguments.
- `getFolderPathStr(...)` joins path parts.
- Timestamp formatting and clearing folders.

</details>

<details>
<summary><b>Bag recording</b></summary>

Starts and stops `ros2 bag` recording as a child process (Boost.Process). Used by the orbit task to collect data for later analysis (Phase 2).

</details>

## Shared types

<details>
<summary><b>Enums</b></summary>

| Enum | Values |
|------|--------|
| `HazardType` | `NONE`, `DUST_PLUME`, `FIBROUS_HAZARD` |
| `TaskType` | `NONE`, `INITIAL_INSPECTION`, `DUST_PLUME_ACTIONS`, `ASBESTOS_ANALYSIS` |
| `CommandType` | `NONE`, `APPROVED`, `CANCELLED` |
| `SeverityLevel` | `NONE`, `LOW`, `MODERATE`, `HIGH`, `VERY_HIGH`, `SEVERE` |
| `OrbitMode` | `NONE`, `RADIAL_IN`, `TANGENT_ALIGN`, `RADIAL_OUT`, `ORBIT` (Phase 2) |
| `Edge` | `NORTH`, `SOUTH`, `EAST`, `WEST` (Phase 2) |

</details>

<details>
<summary><b>Data types</b></summary>

`SimplePose`, `HazardTypePose` (hazard type, pose and diameter, read from the hazard locations JSON), `DustSensorRelayParams`, `TFLookupParams`, `OrbitHazardConfigParams`, `DistanceWithCoOrds`, with JSON conversion functions for the types read from config.

</details>

## Gazebo dust sensor plugin

<details>
<summary><b>DustPlumeSensorPlugin</b></summary>

A C++ Gazebo plugin embedded in the dust plume hazard overlay of the simulation. It publishes dust readings that the `DustSensorRelayPlugin` relays. It only works inside Gazebo, so the unit and integration tests replace it with a stand-in publisher; the end-to-end tests use it as is.

</details>

## Transport interfaces

These live in a separate package, `transport_interfaces`, not in the shared infrastructure package. They are listed here because the nodes and plugins share them.

<details>
<summary><b>Messages, services and actions</b></summary>

| Kind | Definitions |
|------|-------------|
| Messages | `HazardPose`, `MissionStatus`, `MissionCompleted`, `NavigationResult`, `OnArrivalTask`, `BaseReturn`, `StartMission`, `DustSensorResult`, `DensityEstimationResult`, `UIAlerts` |
| Services | `OverrideMission`, `ScanIncident`, `Navigate`, `RotateToPosition`, `InitialInspection`, `OrbitHazard`, `PlumeDensity`, `DensityRegressor` |
| Actions | `RequestPathPlan` |

</details>
