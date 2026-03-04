# Gazebo sim — topic contract checklist

This checklist matches [ROS_Mower_GAZEBO_SIMULATION.md](ROS_Mower_GAZEBO_SIMULATION.md). It shows what the Gazebo setup provides for sim testing.

## Required (base contract)

| Topic | Type | Provided by | Status |
|-------|------|-------------|--------|
| `/cmd_vel` | geometry_msgs/msg/Twist | Bridge ← ROS | ✅ Bridge (ROS → Gazebo) |
| `/odom/raw` | nav_msgs/msg/Odometry | Bridge ← Gazebo diff_drive | ✅ Bridge (Gazebo → ROS) |
| `/base/heartbeat` | std_msgs/msg/Empty | Sim helper node @ 10 Hz | ✅ `sim_helpers_node.py` in gazebo launch |
| `/mower/stall` | std_msgs/msg/Bool | Sim helper node @ 1 Hz (false) | ✅ Same node |
| `/ultrasonic/ranges` | mower_msgs/msg/UltrasonicArray | 6 sensors → aggregator | ⚠️ **Not yet** — see below |

## Optional (EKF / GPS)

| Topic | Type | Provided by | Status |
|-------|------|-------------|--------|
| `/imu/data` | sensor_msgs/msg/Imu | Gazebo IMU sensor → bridge | ✅ Model + bridge |
| `/gps/fix` | sensor_msgs/msg/NavSatFix | Gazebo NavSat sensor → bridge | ✅ Model + bridge (world has spherical_coordinates) |
| `/gps/status` | mower_msgs/msg/GpsStatus | Derived from `/gps/fix` or sim node | ❌ Optional; add small node if safety_manager needs it with GPS |

## What’s in the model and launch

- **Model (`mower_description/models/mower/model.sdf`):** Base, wheels, DiffDrive, 6 ultrasonic (range) sensors, IMU sensor, NavSat (GPS) sensor.
- **World (`mower_empty.sdf`):** Physics, Sensors, **Imu system plugin**, **spherical_coordinates** (for GPS). Set `latitude_deg` / `longitude_deg` to your test origin.
- **Bridge (`config/bridge.yaml`):** `/cmd_vel`, `/odom/raw`, `/imu/data`, `/gps/fix`.
- **Launch (`gazebo.launch.py`):** Gazebo + world, bridge, **sim_helpers_node** (heartbeat + stall).

## Still to do for full contract

1. **`/ultrasonic/ranges`**  
   The 6 Gazebo topics `/model/mower/ultrasonic/0` … `5` exist but are not bridged to ROS. Add either:
   - Bridge entries for the 6 GZ range/LaserScan topics (if type exists in ros_gz_bridge), and an **aggregator node** that subscribes to the 6 ROS topics and publishes `mower_msgs/msg/UltrasonicArray` on `/ultrasonic/ranges` with `range_m[0]` … `range_m[5]`, or  
   - A sim-only node that subscribes to the 6 GZ topics (via bridge or a custom GZ subscriber) and publishes `UltrasonicArray`.

2. **`/gps/status`** (optional)  
   Only needed if you run with `use_gps:=true` and safety_manager uses it. Add a small node that derives `mower_msgs/msg/GpsStatus` from `/gps/fix` (e.g. fix_ok, fix_type, hdop) and publishes on `/gps/status`.

With the above, the sim satisfies the full doc contract (required + optional) for testing.
