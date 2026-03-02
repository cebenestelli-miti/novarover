# ROS Mower — System Architecture (v1)

This document defines the minimal, robust architecture needed to implement the ROS Mower platform
while respecting the core principles:
- High-level autonomy on SBC (e.g., Raspberry Pi)
- Real-time safety authority on microcontroller (Arduino)
- Safety overrides autonomy
- Outdoor GNSS navigation with GNSS+IMU fusion and tolerance to degraded GPS

No ROS distribution, repo, or navigation framework is assumed.

---

## 1) Functional Blocks

### SBC (Autonomy Layer)
Responsibilities:
- Mission management (waypoints, polygons, keep-out zones)
- Localization (GNSS + IMU + wheel odometry fusion)
- Obstacle handling decisions (avoid/stop requests)
- System orchestration and logging

### MCU (Real-Time Control + Safety Layer)
Responsibilities:
- Motor PWM control via H-bridge
- Relay control (engine start/kill, blade enable)
- Ultrasonic acquisition (6 sensors) and publishing (or pre-processing)
- Hall sensor monitoring for stall detection
- Hard safety authority: can stop motion and disable engine/blades independent of SBC state

---

## 2) Data Flow (Logical)

GNSS Fix + IMU + Wheel Odom  ->  Fusion  ->  Filtered Pose/Odom  ->  Mission Executor  ->  Velocity Command
Ultrasonics (6)              ->  Obstacle Guard -> Stop Request  ->  Safety Supervisor -> Safety Stop (authoritative)
Hall Stall                    ->  Safety Supervisor -> Safety Stop (authoritative) + Blade/Engine disable as configured
Heartbeat / Watchdog          ->  Safety Supervisor -> Safety Stop + Engine kill on timeout

---

## 3) Minimal Runtime Components (Node Responsibilities)

### Base Interface (Arduino Bridge)
Inputs:
- Velocity command (cmd_vel)
- Safety stop (authoritative stop)
- Engine/blade command requests (run/stop, blade enable)

Outputs:
- Wheel odometry (raw)
- Ultrasonic array (6 ranges)
- Stall flag
- Heartbeat

### Localization / Fusion
Inputs:
- GNSS fix
- IMU
- Wheel odometry (raw)

Outputs:
- Filtered odometry/pose
- TF/frames consistent with navigation

**Implementation (mower_localization):** Default: forward `/odom/raw` → `/odom` + TF. With `use_ekf:=true`, bringup runs `robot_localization` EKF (config `mower_localization/config/ekf.yaml`) fusing `/odom/raw` and `/imu/data` (e.g. BNO085); planar mode; publishes `/odom` and TF. NEO-M9N: publish `/gps/fix` and `/gps/status`; to fuse GPS, add navsat_transform_node and a second EKF source (see `mower_localization/config/README.md`). Install: `ros-jazzy-robot-localization`.

### Mission Manager
Responsibilities:
- Store and validate missions:
  - Waypoints (GPS coordinates)
  - Polygons for mission areas + keep-out zones
- Execute mission plan:
  - Convert mission to a stream of goals or waypoints
  - Pause/abort on safety state changes

### Ultrasonic Guard
Responsibilities:
- Convert raw 6-range array into:
  - Hazard level
  - Stop request
- Direction-aware logic recommended (e.g., rear sensor only blocks reverse motion)

### Safety Supervisor (Single Source of Truth for STOP)
Responsibilities:
- Publish the authoritative safety stop signal
- Enforce:
  - Watchdog timer (loss of base heartbeat)
  - GNSS loss / stale GNSS data
  - Obstacle stop requests
  - Stall detection response
- Command engine kill / blade disable when required
- Publish a human-readable safety state and reason

---

## 4) Authority Model

- The Safety Supervisor is the only component that asserts the global STOP command.
- The Arduino must also implement a local STOP authority (hardware-level) that does not rely on the SBC.
- Mission execution must never bypass safety; on STOP it must pause/abort and wait for an explicit resume.

---

## 5) Sensor Index Standard (Ultrasonics)

Fixed ordering for the ultrasonic array:

| Index | Position       |
|------:|----------------|
| 0     | front_left     |
| 1     | front_center   |
| 2     | front_right    |
| 3     | left           |
| 4     | right          |
| 5     | rear           |

---

## 6) Safety Behaviors (v1 Suggested Defaults)

- Obstacle within STOP distance: assert STOP immediately.
- GNSS invalid or stale beyond threshold: assert STOP.
- Base heartbeat missing: assert STOP and kill engine.
- Stall while blade enabled: assert STOP; disable blade; optionally kill engine (configurable).

Recovery should require explicit operator/system action (no auto-resume by default).

---

## 7) Mock mode (traceability / deployment safety)

A **mock base interface** is provided so the autonomy stack can run and be tested without the physical robot or Arduino. It publishes `/base/heartbeat`, `/odom/raw`, and optionally `/ultrasonic/ranges` (all clear ranges), and subscribes to `/cmd_vel` and `/safety/stop` so the rest of the system behaves as in deployment.

- **Traceability:** The same bringup launch runs in mock mode (development) and with the real base (deployment); only the `publish_ultrasonic` setting differs.
- **Deployment safety:** When running with real hardware, the real base provides `/ultrasonic/ranges`. To avoid duplicate or conflicting ultrasonic data, either do not run the mock base, or run bringup with ultrasonics disabled on the mock:
  - `ros2 launch mower_base bringup.launch.py publish_ultrasonic:=false`

With `publish_ultrasonic:=false`, the mock still publishes heartbeat and odom for testing; it does not publish `/ultrasonic/ranges`, so the real base (or no ultrasonics) can be used.
