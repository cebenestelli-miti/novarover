# Physical Robot Readiness Audit

**Scope:** Entire ROS workspace (mower_bringup, mower_mission, mower_base, mower_localization, mower_obstacles, ros_mower_core, mower_description, mower_msgs).  
**Purpose:** Determine whether the system is ready for physical robot testing and what is missing.  
**Audit type:** Read-only system audit; no code was modified.

---

## 1. ROS Nodes: Classification

| Node | Package | Classification | Notes |
|------|---------|----------------|--------|
| **mission_manager_node** | mower_mission | **Real hardware capable** | State machine, services, /odom, /safety/state, /gps/status. No sim-only logic. |
| **waypoint_follower_node** | mower_mission | **Real hardware capable** | Subscribes to /odom, /mission/state, /obstacles/hazard_level, /safety/stop, /ultrasonic/ranges; publishes /cmd_vel, /mission/path, /mission/waypoints_markers. Algorithm is frame-agnostic (odom or map). |
| **qgc_mission_converter_node** | mower_mission | **Real hardware capable** | Offline tool: QGC WPL 110 → .wgs84. No runtime dependency on sim. |
| **ultrasonic_guard_node** | mower_obstacles | **Real hardware capable** | Subscribes to /ultrasonic/ranges, /cmd_vel; publishes /obstacles/stop_request, /obstacles/hazard_level. Works with any source of ranges. |
| **safety_manager_node** | ros_mower_core | **Real hardware capable** | E-stop, heartbeat, GPS status, obstacle stop, stall → /safety/stop, /safety/state, /engine/cmd. Full logic implemented. |
| **heartbeat_node** | ros_mower_core | **Real hardware capable** | Standalone /base/heartbeat publisher. Not used when mock or real base runs (base publishes heartbeat). |
| **localization_node** | mower_localization | **Real hardware capable** | Forwards /odom/raw → /odom and TF odom→base_link. No fusion; placeholder for future GPS/IMU. |
| **real_base_interface_node** | mower_base | **Partially implemented** | Structure and topic contract correct. **STUB:** serial_port empty = heartbeat + zero odom; _read_arduino() / _write_arduino() are no-ops. No actual serial or motor driver. |
| **mock_base_interface_node** | mower_base | **Simulation/mock** | Publishes /odom/raw (integrated from /cmd_vel), /ultrasonic/ranges (clear or virtual obstacles), /base/heartbeat. RViz/mock-only. |
| **sim_helpers_node** | mower_description | **Simulation/mock** | Gazebo only: /base/heartbeat, /mower/stall, pose-based /ultrasonic/ranges, cmd_vel gating → /cmd_vel_gz. |
| **ekf_filter_node** | robot_localization (external) | **Real hardware capable** | Launched by bringup when use_ekf:=true. Uses ekf.yaml or ekf_gps.yaml. Not in this repo. |
| **navsat_transform_node** | robot_localization (external) | **Real hardware capable** | Launched when use_ekf:=true use_gps:=true. Converts /gps/fix → /odometry/gps. Not in this repo. |
| **rviz2** | rviz2 (external) | **Tool** | Visualization only; not required for hardware operation. |

**Summary:** Navigation (mission_manager, waypoint_follower), safety (safety_manager), obstacle pipeline (ultrasonic_guard), and localization (localization_node or EKF) are real-node logic. The only **partially implemented** node for hardware is **real_base_interface_node** (no serial/Arduino). **Mock/sim-only:** mock_base_interface_node, sim_helpers_node.

---

## 2. Critical Subsystems

### Navigation

| Component | Exists | Functional | Notes |
|----------|--------|------------|--------|
| waypoint_follower_node | Yes | Yes | Publishes geometry_msgs/Twist to /cmd_vel (remapped to /cmd_vel_nav in bringup). Uses /odom pose; no RViz-specific logic. |
| Mission loader / execution | Yes | Yes | mission_loader.hpp: .waypoints (x y m) or .wgs84 (lat/lon → ENU via farm_origin). mission_manager_node: arm/start/abort/resume/complete; waypoint_follower loads same mission_file + farm_config. |
| Obstacle detection pipeline | Yes | Yes | ultrasonic_guard_node: /ultrasonic/ranges → hazard_level (CLEAR/CAUTION/BLOCKED) → /obstacles/stop_request, /obstacles/hazard_level. waypoint_follower uses both. |
| cmd_vel publishing | Yes | Yes | waypoint_follower publishes real Twist. **Caveat:** bringup.launch.py remaps waypoint_follower to /cmd_vel_nav; real_base_interface subscribes to /cmd_vel. For hardware, need remap or relay so base receives nav commands. |

### Safety

| Component | Exists | Functional | Notes |
|----------|--------|------------|--------|
| safety_manager_node | Yes | Yes | Full implementation. |
| Heartbeat monitoring | Yes | Yes | Subscribes to /base/heartbeat; timeout (base_timeout_sec) + grace period; sets SAFETY_HEARTBEAT_LOSS and /safety/stop. |
| Emergency stop pathway | Yes | Yes | /mower/estop (EStop.engaged) → state ESTOP → /safety/stop true, /engine/cmd engine_run/blade_enable false. |

### Localization

| Component | Exists | Functional | Notes |
|----------|--------|------------|--------|
| Odometry source | Yes (when base provides it) | Partial | /odom/raw from mock_base (sim) or real_base_interface (stub: zero odom until Arduino implemented). |
| TF tree (map / odom / base_link) | Yes | Yes | localization_node: odom → base_link. With EKF+GPS: ekf publishes TF; world_frame can be map so map→odom from EKF/navsat. |
| Pose estimation pipeline | Yes | Yes | Default: /odom/raw → localization_node → /odom + TF. Optional: EKF (odom + IMU [+ GPS]) → /odom + TF. |

---

## 3. Navigation Nodes: Not RViz-Only

- **waypoint_follower_node** uses only standard ROS topics (/odom, /mission/state, /obstacles/hazard_level, /safety/stop, /ultrasonic/ranges) and publishes real **cmd_vel** (Twist). It does not depend on RViz or Gazebo.
- It would work with **real odometry** input: it expects /odom with pose in the same frame as mission waypoints (mission_frame_id, e.g. "odom" or "map").
- **Mission manager** and **mission loader** are file- and parameter-driven; no sim-only assumptions.

---

## 4. GPS and Localization Audit

### 4.1 GPS driver node

- **Not in this repository.** No NMEA driver, UBlox driver, or node that publishes `sensor_msgs/NavSatFix` or `mower_msgs/GpsStatus`.
- Docs and config assume external hardware: **NEO-M9N** (or similar) publishing:
  - `/gps/fix` (sensor_msgs/NavSatFix)
  - `/gps/status` (mower_msgs/GpsStatus) for safety_manager and mission_manager
- **Missing:** A node that (1) reads NMEA or UBlox from serial/USB, (2) publishes /gps/fix, (3) derives and publishes /gps/status (fix_ok, fix_type, hdop, age_sec).

### 4.2 Coordinate conversion (WGS84 → ENU)

- **In repo:** mission_loader converts .wgs84 waypoints to ENU meters using farm_origin (farm_origin.yaml: farm_origin_lat, farm_origin_lon, farm_origin_alt).
- **robot_localization navsat_transform_node:** Converts /gps/fix to local-frame odometry (/odometry/gps) for EKF. Datum/origin config (navsat_transform.yaml): wait_for_datum, magnetic_declination_radians. For farm-fixed map, datum should align with farm_origin (documented in config comments).

### 4.3 Sensor fusion (EKF)

- **In repo:** Launch and config only. bringup.launch.py conditionally launches:
  - **ekf_filter_node** (robot_localization) with ekf.yaml (odom + IMU) or ekf_gps.yaml (odom + IMU + /odometry/gps).
  - **navsat_transform_node** when use_gps:=true.
- **EKF config files (mower_localization):**
  - `config/ekf.yaml` – fuse /odom/raw + /imu/data; world_frame: odom.
  - `config/ekf_gps.yaml` – fuse /odom/raw + /imu/data + /odometry/gps; world_frame: map.
  - `config/navsat_transform.yaml` – navsat_transform_node params (frequency, delay, magnetic_declination_radians, zero_altitude, wait_for_datum).
- **Dependency:** `ros-jazzy-robot-localization` (or equivalent) must be installed.

### 4.4 Localization pipeline output

- With **default** (no EKF): localization_node → /odom (same as /odom/raw) + TF odom→base_link. Frame IDs configurable (e.g. mission_frame_id can be "odom" or "map" for RViz tests).
- With **EKF + GPS:** EKF publishes /odom and TF; world_frame: map so /odom is in map frame; waypoint_follower mission_frame_id should be "map" for WGS84 missions.
- **Stable map→odom:** With GPS fusion, EKF provides consistent pose in map; navsat uses first fix or configured datum. Farm origin alignment is via config (farm_origin.yaml vs navsat datum), not automatic.

### 4.5 Expected topics (GPS fusion)

| Topic | Exists in repo / expected | Notes |
|-------|---------------------------|--------|
| /gps/fix | **Expected from external driver** | sensor_msgs/NavSatFix. Not published by any node in this repo. |
| /gps/status | **Expected from external driver** | mower_msgs/GpsStatus. safety_manager and mission_manager subscribe. Must be populated (e.g. from /gps/fix) for GPS-based safety. |
| /imu/data | **Expected from external driver** | sensor_msgs/Imu. Required for use_ekf:=true. BNO085 assumed in docs. |
| /odom | **Yes** | Published by localization_node or EKF. |
| /tf | **Yes** | From localization_node (odom→base_link) or EKF. |
| /tf_static | **Optional** | Not required for current pipeline. |

### 4.6 What is missing for GPS fusion

1. **GPS driver node** – Publish /gps/fix (and ideally /gps/status) from NMEA or UBlox (e.g. NEO-M9N).
2. **GpsStatus from NavSatFix** – If driver publishes only /gps/fix, a small node or logic to derive mower_msgs/GpsStatus (fix_ok, fix_type, hdop, age_sec) and publish on /gps/status (required by safety_manager and mission_manager).
3. **IMU driver** – Publish /imu/data (e.g. BNO085) for EKF when use_ekf:=true.
4. **Wheel odometry** – /odom/raw from real_base_interface once Arduino/serial is implemented (encoder integration).
5. **Datum/origin alignment** – Ensure navsat_transform_node datum matches farm_origin (lat/lon/alt) for WGS84 missions in map frame.

---

## 5. Hardware Interfaces for Physical Testing

| Interface | Status | Notes |
|-----------|--------|--------|
| **Motor control** | **Missing** | real_base_interface_node has no serial open, _read_arduino/_write_arduino are stubs. Sabertooth/Teensy/Arduino protocol not implemented. Need: serial port open, TX of cmd_vel (and engine_cmd), RX of odom + stall. |
| **Ultrasonic sensors** | **Partially implemented** | real_base_interface_node has ultrasonic_ranges state and publishes /ultrasonic/ranges when publish_ultrasonic true, but values are stub (e.g. 2.0). Actual reading must come from Arduino/serial protocol. |
| **Wheel encoders** | **Missing** | Odometry in real_base is stub (zeros). Encoder integration expected in Arduino/firmware; node should decode and fill odom_x, odom_y, odom_yaw, odom_vx, odom_vz. |
| **IMU** | **Missing in repo** | No IMU driver. External node expected to publish /imu/data (e.g. BNO085). |
| **GPS receiver** | **Missing in repo** | No GPS driver. External node expected to publish /gps/fix (and /gps/status or a bridge). |
| **E-stop hardware** | **Not in repo** | /mower/estop is a topic; physical E-stop button → topic must be wired elsewhere (e.g. RC or a small hardware bridge node). |

---

## 6. Current Dependencies on Mocks / Sim

- **Pose:** All RViz launches use **mock_base_interface_node** (odom from cmd_vel integration). Gazebo uses **ros_gz_bridge** + **sim_helpers_node** for /odom/raw, /base/heartbeat, /ultrasonic/ranges, /cmd_vel_gz.
- **Obstacles:** In RViz tests, obstacles are **simulated** (mock_base virtual circles or sensor noise). ultrasonic_guard and waypoint_follower see the same topic contract as hardware.
- **RViz:** Used only for visualization (path, waypoints, odom). No navigation logic depends on RViz; it can be omitted for physical runs.

So: the stack **depends on** mock/sim only for **who provides** /odom/raw, /base/heartbeat, /ultrasonic/ranges (and in Gazebo, /cmd_vel_gz). Replacing mock with real_base + real sensors and drivers preserves the rest of the pipeline.

---

## 7. Runtime Topics for Hardware Operation

| Topic | Publisher (hardware) | Subscriber(s) | Required |
|-------|------------------------|---------------|----------|
| /cmd_vel or /cmd_vel_nav | waypoint_follower (publishes to /cmd_vel_nav in bringup) | real_base_interface (/cmd_vel) | **Yes** – need routing: /cmd_vel_nav → /cmd_vel for base or change base to /cmd_vel_nav. |
| /odom | localization_node or ekf_filter_node | waypoint_follower, mission_manager | **Yes** |
| /tf | localization_node or EKF | All nodes using TF | **Yes** |
| /tf_static | (optional) | - | No |
| /obstacles/hazard_level | ultrasonic_guard_node | waypoint_follower | **Yes** |
| /mission/state | mission_manager_node | waypoint_follower | **Yes** |
| /mission/waypoints_markers | waypoint_follower | (RViz) | No for operation |
| /base/heartbeat | real_base_interface (or mock/sim_helpers) | safety_manager_node | **Yes** |
| /mower/estop | External (operator/RC) | safety_manager_node | **Yes** |
| /safety/stop | safety_manager_node | waypoint_follower, base (to gate motion) | **Yes** |
| /ultrasonic/ranges | real_base_interface (from hardware) | ultrasonic_guard_node | **Yes** |
| /odom/raw | real_base_interface (from encoders) | localization_node or EKF | **Yes** |
| /gps/fix, /gps/status | External GPS driver | navsat, safety_manager, mission_manager (if GPS used) | For GPS fusion / safety |
| /imu/data | External IMU driver | EKF, navsat | For use_ekf:=true |

---

## 8. Launch Files: Simulation vs Hardware

| Launch | Mode | Base | Localization | Notes |
|--------|------|------|--------------|--------|
| mower_base/bringup.launch.py | **Both** | use_real_base:=false → mock; true → real_base | use_ekf:=false → localization_node; true → EKF (+ optional use_gps) | Single launch supports sim and hardware via args. |
| mower_bringup/bringup.launch.py | **Both** | Forwards to mower_base bringup | Same | Thin wrapper. |
| mower_bringup/rviz_mission_*.launch.py | **Simulation** | mock_base_interface_node | localization_node (odom frame) | RViz + mock only; no real_base, no EKF. |
| mower_description/gazebo.launch.py | **Simulation** | N/A (bridge + sim_helpers) | N/A | Gazebo + bridge + sim_helpers only; bringup launched separately. |

**Conclusion:** **bringup.launch.py** supports hardware mode (use_real_base:=true, use_ekf:=true, use_gps:=true). RViz mission launches are simulation-only (mock base, no EKF). For physical testing, use bringup.launch.py with real base and appropriate localization options.

---

## 9. Nodes to Replace or Modify Before Hardware Test

| Node / component | Action | Reason |
|------------------|--------|--------|
| **real_base_interface_node** | **Implement** | Add serial open, _read_arduino (parse odom, ultrasonics, stall), _write_arduino (cmd_vel, engine_cmd). Replace stub state with real data. |
| **waypoint_follower → cmd_vel routing** | **Configure** | bringup remaps waypoint_follower to /cmd_vel_nav; real_base subscribes to /cmd_vel. Either remap real_base to subscribe to /cmd_vel_nav, or add a relay /cmd_vel_nav → /cmd_vel, or remove remap so follower publishes to /cmd_vel. |
| **GPS driver** | **Add or integrate** | No node in repo publishes /gps/fix and /gps/status. Add new package or integrate existing (e.g. nmea_navsat_driver, ublox_driver) and publish GpsStatus. |
| **IMU driver** | **Add or integrate** | No node in repo publishes /imu/data. Required for use_ekf:=true. |
| **E-stop physical input** | **Wire** | /mower/estop must be driven by physical E-stop (e.g. RC or small bridge node). |

No navigation or safety **core logic** needs to be replaced; only base interface, topic routing, and missing sensor drivers.

---

## 10. Component Status Table

| Component | Status | Simulation / Real | Notes | Required before hardware test |
|-----------|--------|-------------------|--------|-------------------------------|
| waypoint_follower_node | Implemented | Real | Publishes cmd_vel; uses /odom, obstacles, mission state | No change |
| mission_manager_node | Implemented | Real | State machine, geofence, GPS gate | No change |
| mission_loader (.waypoints / .wgs84) | Implemented | Real | farm_origin for ENU | No change |
| ultrasonic_guard_node | Implemented | Real | /ultrasonic/ranges → hazard_level | No change |
| safety_manager_node | Implemented | Real | E-stop, heartbeat, GPS, obstacle, stall | No change |
| localization_node | Implemented | Real | /odom/raw → /odom + TF | No change |
| ekf_node (robot_localization) | Config only | Real | Launched when use_ekf:=true | Install package; provide /imu/data, /odom/raw |
| navsat_transform_node | Config only | Real | Launched when use_gps:=true | Install package; provide /gps/fix, /imu/data |
| real_base_interface_node | Stub | Partial | Heartbeat + zero odom; no serial | Implement serial + Arduino protocol |
| mock_base_interface_node | Implemented | Simulation | Odom + ultrasonics for RViz tests | Replace with real_base in hardware launch |
| sim_helpers_node | Implemented | Simulation | Gazebo heartbeat, stall, ultrasonics, cmd_vel gating | Not used in hardware |
| GPS driver | Missing | - | /gps/fix, /gps/status | Add or integrate driver + GpsStatus |
| IMU driver | Missing | - | /imu/data for EKF | Add or integrate |
| Motor/encoder interface | Missing | - | In Arduino; real_base must read/write serial | Implement in firmware + real_base |
| E-stop physical | Not in repo | - | Topic /mower/estop | Wire hardware to topic |
| cmd_vel routing | Mismatch | - | Follower → /cmd_vel_nav; base → /cmd_vel | Remap or relay for hardware |

---

## 11. Minimum Readiness Checklist for First Physical Drive Test

**Must have (blocking):**

1. **real_base_interface_node** – Serial port open; read odom (and optionally ultrasonics, stall) from Arduino/Teensy; write cmd_vel (and optionally engine_cmd) to hardware; publish /odom/raw and /base/heartbeat.
2. **cmd_vel to base** – Ensure base receives navigation commands (remap /cmd_vel_nav → /cmd_vel for real_base, or run waypoint_follower without remap).
3. **Odometry** – At least wheel odometry on /odom/raw (from real_base or separate node) so localization_node (or EKF) can publish /odom and TF.
4. **E-stop** – Physical E-stop wired to /mower/estop (engaged: true on press) and release before mission start.
5. **Launch** – Use bringup.launch.py with use_real_base:=true, base_serial_port:=/dev/ttyXXX, and mission_file/farm_config as needed. use_ekf:=false is sufficient if only wheel odom is available.

**Should have (for robust operation):**

6. **Ultrasonic sensors** – Real ranges on /ultrasonic/ranges (from real_base or dedicated node) so ultrasonic_guard and waypoint_follower obstacle behavior work.
7. **Heartbeat** – Confirmed that real_base (or equivalent) publishes /base/heartbeat at the rate expected by safety_manager (base_timeout_sec, heartbeat_startup_grace_sec).

**For GPS-based outdoor missions later:**

8. **GPS driver** – Publish /gps/fix (and /gps/status).
9. **IMU driver** – Publish /imu/data.
10. **EKF + GPS** – use_ekf:=true use_gps:=true; navsat datum aligned with farm_origin; mission_frame_id:=map.

---

## 12. Summary and Next Steps

**Already hardware-ready (no code change for first test):**

- Navigation: waypoint_follower, mission_manager, mission_loader.
- Safety: safety_manager (E-stop, heartbeat, obstacle, stall, GPS loss).
- Obstacle pipeline: ultrasonic_guard (consumes /ultrasonic/ranges).
- Localization: localization_node (odom forward + TF); EKF/navsat configs and launch for when sensors exist.
- Topic contracts and message types are defined and used consistently.

**Simulation-only (swap for hardware):**

- mock_base_interface_node → real_base_interface_node (once implemented).
- In Gazebo: sim_helpers_node + bridge; for physical robot, not used.

**Missing for hardware:**

- **In repo:** real_base_interface serial protocol and encoder/ultrasonic parsing; cmd_vel topic routing for bringup.
- **Outside repo:** GPS driver (/gps/fix, /gps/status), IMU driver (/imu/data), physical E-stop wiring to /mower/estop.
- **Firmware:** Arduino/Teensy (or equivalent) for motor control, encoders, ultrasonics, and serial protocol to real_base_interface_node.

**Next development steps before first physical drive test:**

1. Implement **real_base_interface_node** serial protocol (read odom/ultrasonics/stall, write cmd_vel/engine_cmd).
2. Fix **cmd_vel routing** in hardware mode (remap or relay so base receives nav commands).
3. Integrate or implement **wheel odometry** (encoder → /odom/raw) in firmware and real_base.
4. Wire **E-stop** to publish /mower/estop.
5. Optionally add **ultrasonic** reading in firmware and real_base.
6. For GPS missions later: add **GPS driver** and **GpsStatus** publisher; add **IMU driver**; run bringup with use_ekf:=true use_gps:=true and align datum with farm_origin.
