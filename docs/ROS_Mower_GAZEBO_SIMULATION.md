# ROS Mower — Gazebo Simulation

This document describes how to run the ROS Mower autonomy stack in **Gazebo** so you can test missions, safety, and localization without physical hardware. The stack (safety_manager, ultrasonic_guard, mission_manager, waypoint_follower, localization) is unchanged; only the source of base data is replaced by Gazebo.

---

## 1) Purpose and scope

- **Goal:** Run the full autonomy pipeline in simulation: waypoint following, obstacle stop, safety watchdog, optional EKF/IMU/GPS.
- **Out of scope (here):** Motor output on real hardware; that remains the real base interface + Arduino.

Gazebo (with the right model and plugins) must satisfy the **same topic contract** as the mock or real base so that the existing bringup and nodes work without modification.

---

## 2) Topic contract (Gazebo must provide)

The base interface (mock or real) is defined in [ROS_Mower_SYSTEM_ARCHITECTURE.md](ROS_Mower_SYSTEM_ARCHITECTURE.md) and [ROS_Mower_AI_CODER_SPEC.md](ROS_Mower_AI_CODER_SPEC.md). For Gazebo to replace it, the following must hold.

### Required (Gazebo → ROS)

| Topic | Type | Role |
|-------|------|------|
| `/odom/raw` | nav_msgs/msg/Odometry | Wheel odometry from the simulated diff drive. |
| `/base/heartbeat` | std_msgs/msg/Empty | Published at **5–10 Hz** so the safety manager does not trigger heartbeat loss. |
| `/ultrasonic/ranges` | mower_msgs/msg/UltrasonicArray | 6 ranges; index mapping: 0=front_left, 1=front_center, 2=front_right, 3=left, 4=right, 5=rear. |
| `/mower/stall` | std_msgs/msg/Bool | Stall flag; can be `false` in sim if blade stall is not simulated. |

### Required (ROS → Gazebo)

| Topic | Type | Role |
|-------|------|------|
| `/cmd_vel` | geometry_msgs/msg/Twist | Linear.x (m/s), angular.z (rad/s); Gazebo diff_drive plugin subscribes and moves the model. |

### Optional (for EKF / GPS in simulation)

If you launch bringup with `use_ekf:=true` or `use_gps:=true`, you will also need:

| Topic | Type | Role |
|-------|------|------|
| `/imu/data` | sensor_msgs/msg/Imu | For EKF fusion (e.g. Gazebo IMU plugin). |
| `/gps/fix` | sensor_msgs/msg/NavSatFix | For EKF+GPS (e.g. Gazebo GPS plugin). |
| `/gps/status` | mower_msgs/msg/GpsStatus | Used by safety_manager when GPS is enabled; can be published by a small bridge from `/gps/fix` or a sim-specific node. |

---

## 3) Components to add

### 3.1 Robot model (URDF) — `mower_description`

- **Purpose:** Define the robot for Gazebo (and for RViz / TF on the real robot).
- **Contents (minimal for sim):**
  - Base link and geometry (e.g. box or simplified mower footprint).
  - **Differential drive:** Two wheels (left/right) with a Gazebo diff_drive plugin that:
    - Subscribes to `/cmd_vel`.
    - Publishes `/odom/raw` (and optionally TF `odom` → `base_link` if not duplicated by localization).
  - **Optional:** Six ray or sonar sensors (or equivalent) with the same index order as the spec, publishing into a single `mower_msgs/UltrasonicArray` on `/ultrasonic/ranges` (may require a small ROS node to aggregate Gazebo ray topics into one message).
- **Reference:** Ultrasonic index mapping in [ROS_Mower_README.md](ROS_Mower_README.md#-what-is-the-robot?) and [ROS_Mower_AI_CODER_SPEC.md](ROS_Mower_AI_CODER_SPEC.md).

### 3.2 Gazebo world

- A world file (e.g. `.world`) that loads a ground plane and optionally obstacles, buildings, or terrain.
- Spawn the mower model in this world (via launch or Gazebo GUI).

### 3.3 Heartbeat and stall (sim)

- **Heartbeat:** The diff_drive plugin typically does not publish `/base/heartbeat`. Options:
  - A small **timer node** that publishes `std_msgs/msg/Empty` on `/base/heartbeat` at 5–10 Hz, or
  - A custom Gazebo plugin that publishes heartbeat each simulation step.
- **Stall:** Publish `std_msgs/msg/Bool` on `/mower/stall` with `data: false` at a low rate (e.g. 1 Hz), or from a node that only publishes when the value would change.

### 3.4 Ultrasonic array in Gazebo

- Use 6 ray or sonar sensors in Gazebo with the correct positions (front_left, front_center, front_right, left, right, rear).
- If Gazebo publishes 6 separate range topics, add a small **aggregator node** that subscribes to those 6 topics and publishes `mower_msgs/msg/UltrasonicArray` on `/ultrasonic/ranges` with `range_m` ordered by index 0–5.

### 3.5 Optional: IMU and GPS in Gazebo

- **IMU:** Use Gazebo’s IMU plugin to publish `sensor_msgs/msg/Imu` on `/imu/data` (frame_id `base_link`, REP-103).
- **GPS:** Use a Gazebo GPS plugin (or equivalent) to publish `sensor_msgs/msg/NavSatFix` on `/gps/fix`. If safety_manager is used with GPS, also provide `/gps/status` (e.g. derived from fix quality).

---

## 4) Launch strategy

- **Option A — Two processes:**  
  1. **Gazebo:** Start Gazebo, load world, spawn mower model (with diff_drive, optional rays, optional IMU/GPS). Start any helper nodes (heartbeat, ultrasonic aggregator, stall publisher) that are not inside the URDF/plugins.  
  2. **Bringup:** Run `ros2 launch mower_base bringup.launch.py` with **no** mock base and **no** real base (e.g. a dedicated sim launch that only starts safety_manager, ultrasonic_guard, mission_manager, waypoint_follower, localization).

- **Option B — Single sim launch:**  
  One launch file that starts Gazebo, spawns the model, starts the helper nodes, and then starts the same bringup nodes (safety, ultrasonic_guard, mission_manager, waypoint_follower, localization) so that Gazebo is the sole source of `/odom/raw`, `/base/heartbeat`, `/ultrasonic/ranges`, and `/mower/stall`.

Ensure that when running in sim, **mock_base_interface_node** and **real_base_interface_node** are **not** running, to avoid duplicate odom/heartbeat/ultrasonic data.

---

## 5) E-stop and sim

The safety manager starts in E-STOP when `require_estop_release:=true`. To run missions in sim you must either:

- Launch bringup with `require_estop_release:=false`, or  
- Publish once (or via a sim helper node) an e-stop release on `/mower/estop` (topic type `mower_msgs/msg/EStop`, field `engaged: false`).

---

## 6) Summary checklist

| Item | Description |
|------|-------------|
| URDF | `mower_description` with diff_drive (subscribe `/cmd_vel`, publish `/odom/raw`). |
| World | Gazebo world file. |
| Heartbeat | 5–10 Hz on `/base/heartbeat` (timer or plugin). |
| Ultrasonics | 6 ranges → `mower_msgs/UltrasonicArray` on `/ultrasonic/ranges` (indices 0–5). |
| Stall | Publish `/mower/stall` (e.g. always false). |
| Sim launch | Start Gazebo + model + helpers; start bringup **without** mock/real base. |
| E-stop | Set `require_estop_release:=false` or publish estop release on `/mower/estop`. |
| Optional | IMU → `/imu/data`; GPS → `/gps/fix` (+ `/gps/status` if needed). |

Once these are in place, the existing autonomy stack runs in simulation without code changes; only the source of base data is Gazebo instead of the mock or real base.
