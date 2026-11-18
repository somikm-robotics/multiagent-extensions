# DustPlumeActionsNode

[← Back to README](../README.md)

Performs the on-arrival work for a dust plume. It runs two things **at the same time** – relaying readings from the dust sensor and estimating plume density with AI from the camera image – The two produce separate results, and the node reports back only once **both** have been published.

Started by [OnArrivalTaskNode](on_arrival_task_node.md) through `start_dust_plume_actions_task`.

## Behaviour

1. On start, it resets its state and creates two one-shot timers, each in its own callback group, so the two tasks can run concurrently on a multithreaded executor.
2. One timer starts the **dust sensor relay**; the other starts the **density estimation**.
3. Each part reports on its own topic. When both results have been published, the node publishes `dust_plume_actions_complete`.
4. The mission is successful only if **both** parts succeeded.

## Interfaces

| Direction | Name | Type |
|-----------|------|------|
| Subscribes | `start_dust_plume_actions_task` | `StartMission` |
| Subscribes | `dust_sensor_reading` | `DustSensorResult` |
| Subscribes | `estimation_complete` | `DensityEstimationResult` |
| Publishes | `dust_plume_actions_complete` | `MissionStatus` |

## Plugins

| Plugin | Base interface | Responsibility |
|--------|---------------|----------------|
| `DustSensorRelayPlugin` | `DustSensorRelayPluginBase` | Relay dust sensor readings |
| `DustPlumeEstimationPlugin` | `DustPlumeEstimationPluginBase` | Estimate plume density with the Python AI service |

<details>
<summary><b>DustSensorRelayPlugin</b></summary>

- Subscribes to the sensor topic given in config and republishes readings on `dust_sensor_reading` once relaying has been started.
- A reading is relayed when enough of the PM1.0 / PM2.5 / PM10 values exceed the configured threshold.
- Each relayed reading carries a formatted summary (PM values, TSP, opacity, wind, humidity, temperature).
- After the configured number of readings the last one is flagged `completed`, which tells the node the relay is done.

</details>

<details>
<summary><b>DustPlumeEstimationPlugin</b></summary>

- The camera callback stores the latest image under a mutex and notifies a condition variable.
- On start, the plugin waits (up to the configured timeout) for an image. If none arrives, a failed result is published.
- The image is sent to the Python `density_regressor` service (ResNet-18, single density output).
- The returned density (0–1) is mapped to a severity level (low, moderate, high, very high, severe) and published on `estimation_complete` with a message.
- A failed or empty service response is published as a failed result.

</details>

<details>
<summary><b>AI service</b></summary>

All AI stays in Python as the `density_regressor` ROS2 service; the C++ plugin is only a client. In tests the real service is launched, not faked.

</details>

<details>
<summary><b>Gazebo dust sensor</b></summary>

In simulation the readings come from `DustPlumeSensorPlugin`, a C++ Gazebo plugin embedded in the dust plume hazard overlay. In tests it is replaced by a stand-in publisher, since it only runs inside Gazebo.

</details>
