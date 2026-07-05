# ROS2 Applied Extensions
**ROS2 · Nav2 · TurtleBot4 Lite · Arduino · C++**

Post-MSc applied work extending the multi-agent hazard detection system towards real-world validation and production-grade implementation. Each sub-project builds on the simulation-based MSc work, bridging the gap between prototype and deployment.

---

## Sub-Projects

### [Sensor Integration](sensor_integration/README.md)
Real hardware integration of dust (PMS7003) and gas (VOC) sensors into a ROS2-based TurtleBot4 Lite platform. Demonstrates live environmental sensing, real-time data pipelines, and hazard detection capability on physical hardware.

**Status:** Complete

---

### [Nav2 Sim-to-Real](nav2_sim_to_real/README.md)
Autonomous navigation on TurtleBot4 Lite using Nav2 — mapping, localisation, path planning, and a custom orbit-based inspection behaviour. Extends simulation-based Nav2 work from the MSc project to real robot deployment.

**Status:** Work in progress

---

### [Multi-Agent C++ Migration](cpp_migration/README.md)
Migration of Python-based ROS2 agents from the MSc multi-agent system to C++, targeting production-grade performance and real-world deployment readiness.

**Status:** Work in progress

---

## Context

These extensions follow directly from the MSc project:
**[Multi-Agent Robot System for Hazard Detection & Assessment in Ore Mining](https://github.com/yourhandle/multi-agent-hazard-detection)**

The core theme across all projects — autonomous systems detecting and responding to hazards in industrial environments — runs from the MSc simulation work through to real hardware validation here.

---

## Tech Stack

| Category | Technologies |
|----------|-------------|
| Robotics Framework | ROS2, Nav2 |
| Hardware | TurtleBot4 Lite, Raspberry Pi, Arduino |
| Languages | C++ (migration) · Python |
| Sensing | PMS7003 (dust), VOC gas sensor |
| Tools | SLAM Toolbox, Serial Communication |
