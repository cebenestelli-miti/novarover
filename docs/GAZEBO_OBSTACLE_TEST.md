# Gazebo: Full bringup and obstacle-avoidance mission test

Steps to run Gazebo with the mower, bring up the stack (with ultrasonic guard and waypoint follower), release E-stop, and run a mission that drives toward an obstacle so you can test **stop** (blocked), **caution** (slow + steer bias), and rear-sensor-only-on-reverse behavior.

---

## 1. Build and source

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select mower_description mower_mission mower_bringup mower_base
source install/setup.bash
```

(If you already built the whole workspace, `source install/setup.bash` is enough.)

---

## 2. Terminal 1: Start Gazebo

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mower_description gazebo.launch.py
```

Leave this running. The world includes the mower and a red **obstacle box** at (3, 0). Physics update rate is set to 250 Hz to reduce start/pause button flicker on some hosts.

---

## 3. Terminal 2: Bring up the mower stack (sim mode)

Use **odom** frame and **real base** (Gazebo provides `/odom/raw`, sim_helpers provides `/ultrasonic/ranges`). Pass the sim farm config and mission file so mission_manager and waypoint_follower use the obstacle-test waypoints.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mower_bringup bringup.launch.py \
  use_real_base:=true \
  publish_odom:=false \
  publish_ultrasonic:=false \
  use_ekf:=false \
  mission_frame_id:=odom \
  farm_config:=$(ros2 pkg prefix mower_mission)/share/mower_mission/config/farm_origin_sim.yaml \
  mission_file:=$(ros2 pkg prefix mower_mission)/share/mower_mission/config/missions/sim_obstacle_test.waypoints
```

Leave this running. This starts safety_manager, ultrasonic_guard, mission_manager, waypoint_follower, and localization (odom passthrough).

**If `/mission/arm` or `/mission/start` say "waiting for service to become available":** bringup (Terminal 2) must be running and **mission_manager must not have crashed**. Check Terminal 2 for `[ERROR] [mission_manager_node]: process has died`. If you see that, mission_manager failed on startup (e.g. missing `geofence_polygon` in sim config — the sim config is now fixed). Run `ros2 node list` in another sourced terminal; you should see `mission_manager` and `waypoint_follower`. Optional: run `./scripts/diagnose_sim.sh` (after `source install/setup.bash`) to list nodes and mission services.

**If E-stop is released but the robot still doesn't move:** you must run **Terminal 4** (arm + start). The waypoint follower only publishes `/cmd_vel` when the mission is **RUNNING**. Check: `ros2 topic echo /mission/state` (state 2 = RUNNING) and `ros2 topic echo /cmd_vel` (should show non-zero linear.x when running).

---

## 4. Terminal 3: Release E-stop

The robot will not move until E-stop is released.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub /mower/estop mower_msgs/msg/EStop "{stamp: {sec: 0, nanosec: 0}, engaged: false, source: sim}" -r 1
```

Leave this running so E-stop stays released.

---

## 5. Terminal 4: Arm and start the mission

Run **arm** then **start** in the same terminal. If you see `start only from ARMED or IDLE`, the mission_manager was previously changed to **not** transition ARMED→PAUSED on safety; rebuild `mower_mission` and try again.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /mission/arm std_srvs/srv/Trigger {}
ros2 service call /mission/start std_srvs/srv/Trigger {}
```

The robot will drive from (0,0) toward (2,0), then (6,0), etc. The **obstacle box** is at (3,0):

- **CLEAR** (min range ≥ 1 m): normal waypoint following.
- **CAUTION** (0.6 m &lt; min range &lt; 1 m): reduced speed + steering bias away from the obstacle. If the robot doesn’t close at least 2% of the distance to the waypoint in 30 s, it skips that waypoint (caution-stuck skip).
- **BLOCKED** (min range ≤ 0.6 m): waypoint follower stops forward motion. In the band **0.4 m &lt; range ≤ 0.6 m** there is no safety stop, so the robot **pivots** in place toward the clearer side; if still blocked after 15 s it skips to the next waypoint.
- **Safety stop** (min range ≤ 0.4 m): **obstacles/stop_request** true → safety state OBSTACLE → base ignores cmd_vel (no motion, no pivot). After 15 s blocked the waypoint is still skipped.

You can watch `/obstacles/hazard_level` (0=clear, 1=caution, 2=blocked) and `/safety/state` to confirm behavior.

---

## 6. Optional: Monitor topics

In extra terminals (with the same `source` as above):

```bash
# Hazard level: 0 clear, 1 caution, 2 blocked
ros2 topic echo /obstacles/hazard_level

# Safety state (reason when stopped)
ros2 topic echo /safety/state

# Ultrasonic ranges (min/max and per-sensor)
ros2 topic echo /ultrasonic/ranges

# Command actually sent to Gazebo
ros2 topic echo /cmd_vel_gz
```

---

## 7. Abort / stop mission

```bash
ros2 service call /mission/abort std_srvs/srv/Trigger {}
```

## 8. Run the mission again (after it ran once)

The mission state stays RUNNING (or COMPLETE/ABORTED) after the first run. You can only **arm** from **IDLE**. So to run again:

```bash
# If still running, abort first
ros2 service call /mission/abort std_srvs/srv/Trigger {}

# Go back to IDLE (allowed from ABORTED or COMPLETE)
ros2 service call /mission/reset std_srvs/srv/Trigger {}

# Then arm and start as usual
ros2 service call /mission/arm std_srvs/srv/Trigger {}
ros2 service call /mission/start std_srvs/srv/Trigger {}
```

---

## Summary

| Terminal | Role |
|----------|------|
| 1 | Gazebo + bridge + sim_helpers (world, mower, obstacle box, `/odom/raw`, `/ultrasonic/ranges`) |
| 2 | Bringup (safety, ultrasonic_guard, mission_manager, waypoint_follower, odom→/odom) |
| 3 | E-stop released (`/mower/estop` at 1 Hz) |
| 4 | Arm + start mission (and optional abort) |

The **rear sensor** (index 5) only contributes to stop when moving **backward** (`/cmd_vel` linear.x &lt; 0); when moving forward, the rear ultrasonic is ignored by the guard.
