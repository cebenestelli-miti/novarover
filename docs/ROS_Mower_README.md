# ROS Mower

Autonomous outdoor petrol-powered mower platform defined from first principles.

This document specifies:
- The physical robot
- The required software capabilities
- The system constraints
- The architectural philosophy

No implementation stack is assumed at this stage.

---

# 1️⃣ What Is the Robot?

## Mechanical Platform
- Rear-wheel differential drive (independent left/right motors)
- Front passive castor wheels
- Petrol engine for blade drive
- Remote engine start relay
- Remote engine stop (kill) relay
- Blade engagement relay
- Hall sensor for blade RPM / stall detection

## Electronics & Control Hardware
- H-bridge motor controller for rear drive motors
- 6 ultrasonic sensors for obstacle detection (see positions below)
- Microcontroller responsible for:
  - Motor PWM control
  - Relay control
  - Sensor processing
  - Real-time safety enforcement
- Single-board computer responsible for:
  - Autonomy
  - Mission logic
  - Localization
  - Obstacle handling decisions

## Ultrasonic Sensor Positions (fixed index mapping)
The ultrasonic array uses a fixed ordering to avoid ambiguity across the system:

| Index | Position       |
|------:|----------------|
| 0     | front_left     |
| 1     | front_center   |
| 2     | front_right    |
| 3     | left           |
| 4     | right          |
| 5     | rear           |

---

# 2️⃣ What Must the Software System Achieve?

## Mission Execution
- Accept mission GPS coordinates
- Execute waypoint-based missions
- Support polygon boundary recording for outdoor mission areas
- Support keep-out zones within boundaries

## Localization
- GPS-based outdoor navigation
- GPS + IMU fusion for heading stability
- Wheel odometry integration
- Maintain usable localization under temporary GPS degradation

## Mapping (Outdoor Mode)
- Boundary recording for mission areas
- Mission-area storage and reuse
- Mapping is for mission planning, not indoor SLAM

## Obstacle Handling
- Detect obstacles using 6 ultrasonic sensors
- Execute avoidance or stop behaviour
- Immediate halt on critical proximity

## Safety Supervision
- Software watchdog timer
- Failsafe on GPS loss
- Motion halt on localization failure
- Engine kill capability
- Safety state override of mission logic

---

# 3️⃣ What Constraints Exist?
- Outdoor operation only
- Petrol engine-driven blade system
- Rear differential steering
- Front passive castor steering
- Ultrasonic-only obstacle sensing (current version)
- Blade stall detection via hall sensor
- Must tolerate vibration and electrical noise
- Must remain functional during intermittent GPS degradation

---

# 4️⃣ Architectural Principles
- High-level autonomy runs on a single-board computer
- Real-time control and safety run on a microcontroller
- Safety logic has authority over autonomy commands
- Engine kill and motion stop must function independently of autonomy software
- Localization must tolerate degraded GPS conditions
- System must be modular and stack-agnostic
- Hardware safety must not depend solely on software logic

---

# 5️⃣ Mock mode (traceability / deployment safety)

A **mock base interface** is provided so the autonomy stack can run and be tested without the physical robot or Arduino. It publishes `/base/heartbeat`, `/odom/raw`, and optionally `/ultrasonic/ranges` (all clear ranges), and subscribes to `/cmd_vel` and `/safety/stop` so the rest of the system behaves as in deployment.

- **Traceability:** The same bringup launch runs in mock mode (development) and with the real base (deployment); only the `publish_ultrasonic` setting differs.
- **Deployment safety:** When running with real hardware, the real base provides `/ultrasonic/ranges`. To avoid duplicate or conflicting ultrasonic data, either do not run the mock base, or run bringup with ultrasonics disabled on the mock:
  - `ros2 launch mower_base bringup.launch.py publish_ultrasonic:=false`

With `publish_ultrasonic:=false`, the mock still publishes heartbeat and odom for testing; it does not publish `/ultrasonic/ranges`, so the real base (or no ultrasonics) can be used.

---

# 6️⃣ Localization implementation (EKF, BNO085, NEO-M9N)

- **Without IMU (default):** `ros2 launch mower_base bringup.launch.py` — simple node forwards `/odom/raw` → `/odom` + TF.
- **With BNO085 (IMU):** `ros2 launch mower_base bringup.launch.py use_ekf:=true` — EKF fuses `/odom/raw` and `/imu/data`; config in `mower_localization/config/ekf.yaml`; planar mode; install `ros-jazzy-robot-localization`.
- **With BNO085 + NEO-M9N (GPS):** `ros2 launch mower_base bringup.launch.py use_ekf:=true use_gps:=true` — EKF fuses `/odom/raw`, `/imu/data`, and `/odometry/gps`; `navsat_transform_node` converts `/gps/fix` → `/odometry/gps`. Config: `ekf_gps.yaml`, `navsat_transform.yaml`; set `magnetic_declination_radians` for your location.
- **Expected topics:** BNO085 → `/imu/data` (Imu, frame `base_link`, REP-103). NEO-M9N → `/gps/fix` (NavSatFix), `/gps/status` (GpsStatus for safety).

---

# 7️⃣ Gazebo simulation

The autonomy stack can be run in **Gazebo** so you can test missions and safety without hardware. Simulation does not replace the mock base conceptually: Gazebo (with the right plugins) *provides* the same topic contract as the real or mock base (`/odom/raw`, `/base/heartbeat`, `/ultrasonic/ranges`, `/mower/stall`, and subscription to `/cmd_vel`). You then run the normal bringup **without** the mock or real base so that Gazebo is the only source of odometry, heartbeat, and ultrasonics.

**What you need for Gazebo:**
- **Robot model (URDF):** `mower_description` — differential-drive base, optional ray/sonar plugins for the 6 ultrasonics.
- **Gazebo world:** A world file (e.g. outdoor or test pad).
- **Plugins / bridge:** Publish `/odom/raw` and subscribe to `/cmd_vel` (e.g. `libgazebo_ros_diff_drive.so`); publish `/base/heartbeat` at 5–10 Hz; publish 6 ranges as `mower_msgs/UltrasonicArray` on `/ultrasonic/ranges`; publish `/mower/stall` (e.g. always false in sim). Optionally add IMU and GPS plugins if you want to test EKF/GPS in simulation.
- **Sim launch:** Spawn the model in Gazebo and run the above; start the rest of the stack via bringup with mock/real base disabled (or a dedicated sim bringup that omits base nodes).

**Detailed requirements, topic mapping, and step-by-step setup are in [ROS_Mower_GAZEBO_SIMULATION.md](ROS_Mower_GAZEBO_SIMULATION.md).**

---

This document defines the robot and its required capabilities.
Software stack selection will be made separately after validating these system requirements.
