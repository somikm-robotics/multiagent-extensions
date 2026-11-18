# OnArrivalTaskNode

[← Back to README](../README.md)

The central point for all hazard tasks. Depending on the hazard type and task type received from [MissionHandlerNode](mission_handler_node.md), it forwards the request so the right task starts. It also listens for the task to finish and publishes the outcome so the mission handler can act on it.

## Behaviour

On `on_arrival_task`, the node looks at the hazard type and task type:

| Hazard | Task | What happens |
|--------|------|--------------|
| Fibrous hazard | Initial inspection | `InitialInspectionPlugin` spins the robot and captures images |
| Fibrous hazard | Asbestos analysis (orbit) | Orbit plugin circles the hazard – **Phase 2** |
| Dust plume | Dust plume actions | Starts [DustPlumeActionsNode](dust_plume_actions_node.md) through `start_dust_plume_actions_task` |

When the task finishes, the node publishes `on_arrival_task_mission_status` (hazard type, task type, message, success), which MissionHandlerNode uses to start the return to base.

## Interfaces

| Direction | Name | Type |
|-----------|------|------|
| Subscribes | `on_arrival_task` | `OnArrivalTask` |
| Subscribes | `inspection_complete` | `MissionCompleted` |
| Subscribes | `orbit_completed` | `MissionCompleted` |
| Subscribes | `dust_plume_actions_complete` | `MissionStatus` |
| Publishes | `start_dust_plume_actions_task` | `StartMission` |
| Publishes | `on_arrival_task_mission_status` | `MissionStatus` |

## Plugins

| Plugin | Base interface | Responsibility |
|--------|---------------|----------------|
| `InitialInspectionPlugin` | `InitialInspectionPluginBase` | Spin in place and save images at fixed angle steps |
| Orbit plugin (`Nav2OrbitHazardManagerPlugin` or `TwistCommanderOrbitHazardPlugin`, chosen in config) | `OrbitHazardManagerPluginBase` | Orbit the hazard – Phase 2 |

<details>
<summary><b>InitialInspectionPlugin</b></summary>

- Creates a new time-stamped folder for each inspection.
- Publishes a slow rotation command (`cmd_vel`, topic from config) at 10 Hz.
- Listens to odometry for the current yaw and saves a camera image at the first reading and then every 30° of rotation, until the configured number of images has been saved.
- Then stops the robot and publishes `inspection_complete` with the number of images saved.
- Camera and velocity topics come from config.

</details>

<details>
<summary><b>Orbit plugins (Phase 2)</b></summary>

The node already loads an orbit plugin (Nav2-based or twist-based, selected in config) and handles `orbit_completed`. The orbit task is not part of the Phase 1 mission flow.

</details>

<details>
<summary><b>Why the dust plume task is delegated</b></summary>

Dust sensing and AI estimation run in parallel and need their own callback groups, so they live in a separate node. OnArrivalTaskNode only starts it and forwards its result.

</details>
