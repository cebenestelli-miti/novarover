# ROS Mower — AI Coder Spec (Implementation Contract)

Use this document as the **source of truth** for an AI coding agent implementing the system.
Do not invent new topic names, reorder arrays, or bypass safety authority.

---

## 1) Package List (suggested)
- mower_msgs
- mower_base
- mower_safety
- mower_obstacles
- mower_mission
- mower_localization
- mower_description
- mower_bringup

(Names are suggestions; keep responsibilities and interfaces the same even if naming differs.)

---

## 2) Topic Contracts (must match)

### Motion
- /cmd_vel : geometry_msgs/msg/Twist
  - linear.x (m/s), angular.z (rad/s)
- /safety/stop : std_msgs/msg/Bool
  - TRUE = stop immediately; base must ignore non-zero cmd_vel while TRUE

### Base Supervision
- /base/heartbeat : std_msgs/msg/Empty
  - Published at 5–10 Hz by base interface (Arduino bridge)

### Odometry / Sensors
- /odom/raw : nav_msgs/msg/Odometry
- /odom : nav_msgs/msg/Odometry — consumed by waypoint follower and mission (geofence). **Topic /odom may be in map frame when GPS is enabled.** Localization (or EKF with `world_frame: map`) sets `header.frame_id` to the mission frame (e.g. `"map"`); the waypoint follower only uses odom when `header.frame_id` matches its `mission_frame_id` param (default `"map"`).
- /imu/data : sensor_msgs/msg/Imu
- /gps/fix : sensor_msgs/msg/NavSatFix

### Ultrasonics (Option A)
- /ultrasonic/ranges : mower_msgs/msg/UltrasonicArray

Fixed index mapping:
| idx | name         |
|---:|--------------|
| 0  | front_left   |
| 1  | front_center |
| 2  | front_right  |
| 3  | left         |
| 4  | right        |
| 5  | rear         |

### Stall
- /mower/stall : std_msgs/msg/Bool

### Engine/Blade Control
- /engine/cmd : mower_msgs/msg/EngineCmd

### Status
- /gps/status : mower_msgs/msg/GpsStatus
- /safety/state : mower_msgs/msg/SafetyState
- /mission/state : mower_msgs/msg/MissionState

---

## 3) Message Definitions (create in mower_msgs)

### UltrasonicArray.msg
std_msgs/Header header
float32[6] range_m
float32 min_range_m
float32 max_range_m

### EngineCmd.msg
bool engine_run
bool blade_enable
bool engine_start_pulse
uint8 throttle_mode   # 0=NONE (v1)
string reason         # optional

### GpsStatus.msg
bool fix_ok
uint8 fix_type        # 0 none, 1 2D, 2 3D, 3 RTK float, 4 RTK fixed (reserved)
float32 hdop
float32 age_sec

### SafetyState.msg
uint8 state           # enum below
string reason
bool stop_asserted
bool engine_run_allowed
bool blade_allowed

# Suggested SafetyState enum values (constants in code)
# 0 RUNNING
# 1 STOPPED
# 2 FAULT
# 3 E_STOP
# 4 GPS_LOSS
# 5 OBSTACLE
# 6 STALL
# 7 HEARTBEAT_LOSS

### MissionState.msg
uint8 state           # enum below
string reason

# Suggested MissionState enum values (constants in code)
# 0 IDLE
# 1 ARMED
# 2 RUNNING
# 3 PAUSED
# 4 ABORTED
# 5 COMPLETE

---

## 4) Safety Supervisor Rules (must implement)

Inputs that can trigger STOP:
1) Obstacle stop request (/obstacles/stop_request) OR derived directly from ultrasonics
2) GPS loss/stale:
   - gps_status.fix_ok == false for > GPS_LOSS_SEC
   - gps_status.age_sec > GPS_MAX_AGE_SEC
   - gps_status.hdop > HDOP_MAX (v1: treat as stop or pause; default stop)
3) Base heartbeat missing for > BASE_TIMEOUT_SEC
4) Stall detected while blade enabled

Outputs:
- Publish /safety/stop TRUE as the authoritative stop
- Publish /safety/state with a specific reason
- Publish /engine/cmd to disable blade and/or kill engine depending on config

Recovery:
- No auto-resume by default. Resume requires explicit external action and cleared hazards.

---

## 5) Ultrasonic Guard (must implement)

Input:
- /ultrasonic/ranges (UltrasonicArray)

Outputs:
- /obstacles/stop_request : std_msgs/msg/Bool
- /obstacles/hazard_level : std_msgs/msg/UInt8

Rules (v1 defaults):
- If any range_m[i] < STOP_DIST_M -> stop_request TRUE
- Recommend direction-aware gating:
  - rear sensor only blocks reverse motion (optional in v1 if velocity is available)
Implementation for obstacle go-around:
- Use `hazard_level` to represent graded states such as:
  - 0 = CLEAR (all ranges >= CAUTION_DIST_M)
  - 1 = CAUTION (at least one range in [STOP_DIST_M, CAUTION_DIST_M), none < STOP_DIST_M)
  - 2 = BLOCKED (at least one range < STOP_DIST_M)
- SafetySupervisor continues to use only `/obstacles/stop_request` to assert STOP;
  hazard levels are for higher-level behaviors (e.g., local go-around
  maneuvers in the mission/waypoint follower logic that detour around obstacles
  while remaining within the mission area / outside keep-out zones).

Suggested waypoint follower behavior using hazard_level and the 6 ultrasonics:
- When `hazard_level == CLEAR`: follow the nominal waypoint trajectory.
- When `hazard_level == CAUTION` (front sensors close but not in STOP zone):
  - Reduce forward speed.
  - Bias `cmd_vel.angular.z` away from the closer front obstacle: if front_center
    is near and front_right is clearer than front_left, turn right (and vice versa).
  - Use side sensors to avoid scraping along obstacles while going around.
- When `hazard_level == BLOCKED` or `/safety/stop` is TRUE: publish zero `cmd_vel`
  and wait for the guard/safety to clear the stop.

---

## 6) Parameters (defaults for v1)

STOP_DIST_M: 0.60
CAUTION_DIST_M: 1.00
GPS_LOSS_SEC: 2.0
GPS_MAX_AGE_SEC: 1.0
HDOP_MAX: 2.5
BASE_TIMEOUT_SEC: 0.5
STALL_DEBOUNCE_SEC: 0.5
ENGINE_KILL_ON_FAULT: true

---

## 7) AI Coder Prompt (copy/paste)

Implement ROS 2 packages for mower_msgs, mower_base, mower_safety, mower_obstacles, mower_bringup, mower_description.
Follow these contracts exactly:
- /cmd_vel Twist to base driver
- /safety/stop Bool is authoritative; base must ignore non-zero cmd_vel when true
- /base/heartbeat Empty at 10 Hz from base interface
- /ultrasonic/ranges uses mower_msgs/UltrasonicArray with fixed index mapping:
  [0 front_left, 1 front_center, 2 front_right, 3 left, 4 right, 5 rear]
- SafetySupervisor asserts stop on: obstacle, GPS loss/stale, heartbeat timeout, stall
- Create messages: UltrasonicArray, EngineCmd, GpsStatus, SafetyState, MissionState
- Provide launch files in mower_bringup to start base interface (with mock mode), ultrasonic_guard, safety_supervisor
- Add a mock hardware mode so it runs without Arduino (publishes heartbeat + fake odom + fake ultrasonics)
Do not introduce repo-specific assumptions or rename topics.

---

## 8) Mock mode (traceability / deployment safety)

A **mock base interface** is provided so the autonomy stack can run and be tested without the physical robot or Arduino. It publishes `/base/heartbeat`, `/odom/raw`, and optionally `/ultrasonic/ranges` (all clear ranges), and subscribes to `/cmd_vel` and `/safety/stop` so the rest of the system behaves as in deployment.

- **Traceability:** The same bringup launch runs in mock mode (development) and with the real base (deployment); only the `publish_ultrasonic` setting differs.
- **Deployment safety:** When running with real hardware, the real base provides `/ultrasonic/ranges`. To avoid duplicate or conflicting ultrasonic data, either do not run the mock base, or run bringup with ultrasonics disabled on the mock:
  - `ros2 launch mower_base bringup.launch.py publish_ultrasonic:=false`

With `publish_ultrasonic:=false`, the mock still publishes heartbeat and odom for testing; it does not publish `/ultrasonic/ranges`, so the real base (or no ultrasonics) can be used.

---

## 9) Localization (EKF option and sensors)

- **Default (`use_ekf:=false`):** Simple localization node forwards `/odom/raw` → `/odom` and broadcasts TF `odom` → `base_link`.
- **With IMU (`use_ekf:=true`):** EKF fuses `/odom/raw` and `/imu/data` (BNO085). Config: `ekf.yaml`.
- **With IMU + GPS (`use_ekf:=true use_gps:=true`):** `navsat_transform_node` converts `/gps/fix` → `/odometry/gps`; EKF fuses odom + IMU + `/odometry/gps`. Config: `ekf_gps.yaml`, `navsat_transform.yaml`.
- **Dependency:** `ros-jazzy-robot_localization`. `mower_base` has `exec_depend` on `robot_localization`.
- **Sensors:** **BNO085** → `/imu/data` (Imu, frame `base_link`, REP-103). **NEO-M9N** → `/gps/fix` (NavSatFix), `/gps/status` (GpsStatus for safety).
- **Usage:** Default: no fusion. IMU: `use_ekf:=true`. IMU+GPS: `use_ekf:=true use_gps:=true`.
- **Farm-fixed map:** With GPS, EKF uses `world_frame: map`. Waypoints and robot pose are in the same frame (map). NavSat datum should match `farm_origin` (see below) so /odom is in map.

---

## 10) Mission storage, formats, and startup gates

- **.waypoints (backward compatible):** One "x y" (meters) per line. Comments with #. No conversion. **.waypoints are in mission_frame_id** (e.g. "odom" or "map"); when using farm config that is "map".
- **.wgs84 (phone logger):** Optional first line `# origin: lat lon`. Then one "lat lon" per line. Converted to ENU meters at load using `farm_origin_*` (or file origin if present); result is in the same frame as mission (map when using farm origin). Extension `.wgs84` selects this format.
- **Farm-fixed origin:** Params `farm_origin_lat`, `farm_origin_lon`, (optional) `farm_origin_alt`. In bringup/config (e.g. `farm_origin.yaml`). Used when loading .wgs84 so waypoints are in the same map frame as localization.
- **Mission frame:** Waypoint follower and mission manager use waypoints in mission_frame_id (e.g. "map" with farm config). Localization must publish /odom with that frame_id so pose and waypoints are aligned.
- **Waypoint loader + follower aligned:** .waypoints files are implicitly in **mission_frame_id** (when using farm config, that is "map"). .wgs84 is converted to ENU in that same frame at load. The follower only uses /odom when `header.frame_id == mission_frame_id`, so waypoints and pose are always in the same frame.
- **Startup gates (before mission/start succeeds):**  
  - **GNSS quality gate:** If `require_gnss_quality` is true, refuse start unless /gps/status has fix_ok and fix_type >= min_fix_type (e.g. 2 = 3D).  
  - **Geofence gate (optional):** Geofence polygon is defined in **mission_frame_id** (map frame when using farm config). The pose used for the check **must** be from an /odom message where `header.frame_id == mission_frame_id`; otherwise you can get false fails or (worse) false passes. Mission manager refuses start if frame_id does not match before evaluating inside/outside.
