# Supervisory agent

[← Back to README](../README.md)

The operator's side of the system: a web dashboard and the ROS2 nodes behind it. It lets the operator select a location to scan, see status messages, and approve or cancel a mission.

 In the MSc project, scanning a location triggered the aerial agent to look for hazards. Since the aerial agent has not been migrated, here the scan instead looks the location up in a hazard locations config. The config file stores the hazard location details. Only this scan and the config stand in for the aerial agent; the rest of the supervisory agent is part of the MSc project. As it is not a core part of the C++ migration project, it has no unit tests.

## How it works

1. The operator picks a location in the dashboard and presses **Scan Incident**.
2. If the location holds a hazard, a `HazardPose` is published and the ground agent starts its mission. Otherwise the dashboard shows "No hazard found".
3. The dashboard shows status messages as the mission progresses. While the override window is open, **Approve** and **Cancel** are enabled so the supervisor can act, if required.
4. After the robot returns to base and parks, scanning is enabled again.

## Nodes

| Node | Responsibility |
|------|---------------|
| `MissionOverrideNode` | Handles scan requests (publishes `hazard_detected`) and forwards approve / cancel to the ground agent's `override_mission` service |
| `AlertReceiverNode` | Collects mission information from the ground agent and publishes it to the UI as `UIAlerts` |
| `SupervisoryAgentBridgeNode` | Connects the web server to ROS2: queues alerts for the UI and calls the scan and override services |

The web server (an HTTP server on port 8000) runs in the same process as the bridge node, with ROS2 spinning on its own thread.

<details>
<summary><b>MissionOverrideNode</b></summary>

- Service `/scan/incident` (`ScanIncident`): looks up the selected location in the hazard locations config. If it holds a hazard, publishes `hazard_detected` with its type, location and diameter and accepts the scan; if not, responds "No hazard found".
- Service `/ui/override/mission` (`OverrideMission`): forwards the operator's command to the ground agent's `override_mission` service and returns its response.

</details>

<details>
<summary><b>AlertReceiverNode</b></summary>

Subscribes to hazard detected, intermediary mission status, return to base, density estimation result and dust sensor reading, and publishes each as a `UIAlerts` message on `/supervisor/alert`. The message tells the UI which controls to enable:

| Source | Effect in the UI |
|--------|-----------------|
| Hazard detected | Status shown; approve / cancel enabled |
| Mission status | Message shown; analysis controls enabled if analysis is required (hidden in Phase 1) |
| Return to base | Status shown; scan enabled |
| Dust sensor reading, density estimate | Message shown |

</details>

<details>
<summary><b>SupervisoryAgentBridgeNode</b></summary>

- Receives `UIAlerts` and queues them (mutex and condition variable) for two streams: a status stream and a history stream.
- Calls the `/scan/incident` and `/ui/override/mission` services on behalf of the web server and returns success and message.
- Provides the list of location names for the dashboard.

</details>

<details>
<summary><b>Web server and dashboard</b></summary>

| Endpoint | Purpose |
|----------|---------|
| `GET /` and static files | Dashboard HTML, JavaScript and CSS |
| `GET /mines/locations` | Location names for the dropdown |
| `GET /status/stream` | Server-sent events: current status line |
| `GET /history/stream` | Server-sent events: message history and which buttons are enabled |
| `POST /scan/incident` | Scan the selected location |
| `POST /ui/override/mission` | Approve or cancel |

The dashboard is plain HTML, JavaScript and CSS: a message history area, Approve / Cancel buttons, a scan section with a location dropdown, and a status bar. The JavaScript only wires up the page, the event streams and the requests; the logic stays in C++.

</details>

<details>
<summary><b>Hazard locations</b></summary>

Locations come from the hazard locations JSON (see the README configuration section). Each entry has a hazard type, position and estimated diameter; a location with no hazard gives "No hazard found" when scanned.

</details>
