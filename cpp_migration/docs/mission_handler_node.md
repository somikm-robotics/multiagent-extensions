# MissionHandlerNode

[← Back to README](../README.md)

The central orchestrator of the ground agent. It owns the mission from the moment a hazard is reported until the robot is parked at base again. It does no navigation or path planning itself: it delegates to two plugins and hands the on-arrival work to [OnArrivalTaskNode](on_arrival_task_node.md).

## Mission flow

1. A hazard arrives on `hazard_detected`. If the agent is ready, a mission starts and the **override window** opens; its timeout is set in config.2. The operator approves or cancels through the `override_mission` service. If nothing arrives before the timer fires, the mission proceeds.
3. The node requests a path to the hazard from the path planner plugin, then hands the waypoints to the navigation plugin to drive.
4. On arrival (`reached_target`) the node publishes `on_arrival_task`.
5. When the on-arrival task reports back (`on_arrival_task_mission_status`), the node requests a path back to base from the path planner plugin and hands the waypoints to the navigation plugin again.
6. On reaching base, the node asks the navigation plugin to rotate the robot to the parking heading (`completed_rotation`). It then publishes the return-to-base status and becomes ready for the next mission.

Status updates for the UI are published on `intermediary_mission_status` throughout.

## Interfaces

| Direction | Name | Type |
|-----------|------|------|
| Subscribes | `hazard_detected` | `HazardPose` |
| Subscribes | `reached_target` | `NavigationResult` |
| Subscribes | `on_arrival_task_mission_status` | `MissionStatus` |
| Subscribes | `completed_rotation` | `MissionCompleted` |
| Publishes | `on_arrival_task` | `OnArrivalTask` |
| Publishes | `intermediary_mission_status` | `MissionStatus` |
| Publishes | `return_to_base_status` | `BaseReturn` |
| Service | `override_mission` | `OverrideMission` |

## Plugins

| Plugin | Base interface | Responsibility |
|--------|---------------|----------------|
| `Nav2PathPlannerPlugin` | `PathPlannerPluginBase` | Plans a path through Nav2 and returns waypoints |
| `Nav2NavigationPlugin` | `NavigationPluginBase` | Drives the waypoints and performs the parking rotation |

<details>
<summary><b>Mission states</b></summary>

The node tracks its state with `MissionStatus`: `READY`, `STARTED`, `APPPROVEDINITIAL`, `INPROGRESS`, `PENDINGANALYSIS`, `APPROVEDANALYSIS`, `CANCELLED`, `CRITICAL`, `UNKNOWN`.

- A new hazard is accepted only in `READY` (or `UNKNOWN`, after an earlier error). Otherwise the UI is told processing is already in progress.
- `CANCELLED` returns to `READY` once the override timer is cleaned up.
- `CRITICAL` means the robot may be stuck on the way or at the destination; the status message tells the operator the application needs a restart.

</details>

<details>
<summary><b>Override window</b></summary>

- On a new hazard the node starts a wall timer for the configured number of minutes.
- **Approve** – the timer is cancelled and the path request is sent immediately.
- **Cancel** – the mission is aborted and the node returns to ready.
- **Timeout** – the mission proceeds as if approved.

</details>

<details>
<summary><b>Return to base and parking</b></summary>

- The return pose is the base pose captured when the hazard was received.
- After the on-arrival task succeeds, the same path planner and navigation plugins are used for the way back.
- Once at base, the node looks up the current heading from TF and asks the navigation plugin to spin to the parking heading (shortest turn). A timeout on the TF lookup reports parking as failed.

</details>

<details>
<summary><b>Failure handling</b></summary>

- Path planning or navigation failure, or an on-arrival task failure, sets the state to `CRITICAL` (or `UNKNOWN` before the robot has left) and publishes the reason to the UI.
- Invalid hazard types are rejected and reported.

</details>

<details>
<summary><b>Phase 1 / Phase 2</b></summary>

In Phase 1 the fibrous hazard mission ends after the initial inspection. The orbit step (asbestos analysis) is added in Phase 2.

</details>

## Plugin pages

<details>
<summary><b>Nav2PathPlannerPlugin</b></summary>

- Receives the `RequestPathPlan` goal from the node (target pose and whether it is a return to base).
- Sends `ComputePathToPose` to Nav2 (start taken from the current robot pose).
- Down-samples the returned path (minimum spacing between waypoints, last pose always kept), drops a first pose that coincides with the robot, and sets each waypoint's orientation towards the next.
- Returns the result to the node through the `RequestPathPlan` result callback; failures (server timeout, goal rejected, Nav2 error) are returned as failed results.

</details>

<details>
<summary><b>Nav2NavigationPlugin</b></summary>

- **Navigation** – sends the waypoints to Nav2 `NavigateThroughPoses` and publishes `reached_target` with success or failure.
- **Parking rotation** – sends a Nav2 `Spin` goal for the shortest turn to the target heading and publishes `completed_rotation`.
- Exposes both as services (`navigate_to_pose`, `complete_rotation`) as well as direct calls from the node.

</details>
