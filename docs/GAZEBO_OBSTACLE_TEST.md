# Gazebo: Full bringup and obstacle-avoidance mission test

> **WSL2 note:** `gpu_lidar` sensors require a GPU renderer and do not function in WSL2.
> Obstacle detection is handled in software by `sim_helpers_node`, which uses ray-circle
> intersection against known obstacle positions derived from `/odom/raw`. No code changes
> are needed to run on real hardware — the software sensor is sim-only.

## Obstacle avoidance algorithm (perpendicular-drive with post-clear)

The `waypoint_follower_node` implements a reactive perpendicular-drive strategy:

1. **NORMAL** — robot drives straight toward the current waypoint at full speed.
2. **AVOID triggered** — sensor 1 (front-center, 0°) reads closer than `go_around_trigger_dist_m`
   (2.0 m).  Angled sensors 0 and 2 (±45°) have a near-disabled threshold (0.25 m = min_range)
   to prevent false triggers when passing the obstacle laterally.  Requires **3 consecutive**
   triggered ticks (0.3 s) before AVOID starts — filters single-tick look-back false positives.
3. **AVOID phase A – TURNING** — robot rotates in the chosen direction (picks the clearer side)
   until front is clear and heading is within ~11° of the perpendicular heading.
4. **AVOID phase B – DRIVING** — robot drives forward at `go_around_speed_ms` (0.25 m/s) while
   holding the perpendicular heading (P-control). Tracks when the wall sensor first detects the
   obstacle (`side_saw_`), then detects it going clear.
5. **AVOID phase C – POST_CLEAR** — after the wall sensor clears, robot continues driving straight
   (same perpendicular heading) for `go_around_post_clear_m` (2.5 m). This guarantees the robot
   is far enough north of the obstacle before resuming the original waypoint heading — prevents
   the exit path from clipping the obstacle's bounding sphere.
6. **AVOID exit** — when post_clear distance is reached and front sensors are clear: resume NORMAL.

Key design decisions:
- Side sensors (3 and 4): **excluded from `stop_request`** (never cause safety stop while wall-following).
- Obstacle sphere model radius: **0.25 m** (= box half-side, not half-diagonal) to match actual
  box face geometry and prevent over-predicted stop_request from the lateral pass.
- Heartbeat timeout: **5.0 s** (was 0.5 s) — WSL2 Gazebo physics can pause briefly during rapid
  turns, causing transient heartbeat gaps in `sim_helpers_node`.

---

## 1. Build and source

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mower_description mower_mission mower_base mower_obstacles ros_mower_core
source install/setup.bash
```

(If you already built the whole workspace, `source install/setup.bash` is enough.)

---

## 2. Terminal 1: Start Gazebo (leave running)

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mower_description gazebo.launch.py cmd_vel_in_topic:=/cmd_vel_nav
```

Leave this running. The world loads the mower at (0, 0) and a **red obstacle box** at (5, 0).
The launch starts with `-r` (auto-play) and starts the bridge + `sim_helpers_node` 3 s after
Gazebo.

> **Stale process tip:** if the first odom check shows x >> 0, run
> `pkill -9 -f "gz sim"` and restart.

---

## 3. Terminal 2: Bringup (leave running)

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mower_base bringup.launch.py \
  use_sim:=true \
  publish_ultrasonic:=false \
  mock_publish_odom:=false \
  mission_frame_id:=odom \
  farm_config:=$(ros2 pkg prefix mower_mission)/share/mower_mission/config/farm_origin_sim.yaml \
  mission_file:=$(ros2 pkg prefix mower_mission)/share/mower_mission/missions/sim_obstacle_test.waypoints
```

Key flags:
- `mock_publish_odom:=false` — Gazebo bridge owns `/odom/raw`; no dual-publisher.
- `publish_ultrasonic:=false` — `sim_helpers_node` provides `/ultrasonic/ranges`.

### Square waypoint sanity test (no obstacle logic required)

To validate frame + waypoint interpretation with a simple 1 m square mission:

```bash
ros2 launch mower_base bringup.launch.py \
  use_sim:=true \
  publish_ultrasonic:=false \
  mock_publish_odom:=false \
  mission_frame_id:=odom \
  farm_config:=$(ros2 pkg prefix mower_mission)/share/mower_mission/config/farm_origin_sim.yaml \
  mission_file:=$(ros2 pkg prefix mower_mission)/share/mower_mission/missions/sim_square_1m.waypoints
```

---

## 4. Terminal 3: Release E-stop (one-shot)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /mower/estop mower_msgs/msg/EStop "{engaged: false, source: 'operator'}"
```

Wait ~3 s for `safety_manager` to log `State transition -> IDLE`.

---

## 5. Terminal 4: Arm and start the mission

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service call /mission/arm   std_srvs/srv/Trigger {}
ros2 service call /mission/start std_srvs/srv/Trigger {}
```

Expected sequence (watch bringup terminal):
1. **Reached WP 0 (2.00, 0.00)**
2. **AVOID START dir=LEFT wall_sensor=4** — obstacle detected at ~2 m
3. Robot arcs left (north), wall-follows the obstacle
4. **AVOID COMPLETE → resuming NORMAL** (obstacle cleared from side sensor)
5. May cycle 2–4× as the robot re-detects during final approach — this is normal
6. **Reached WP 1 (6.00, 0.00)** — robot is now *past* the obstacle
7. WP 2 → WP 3 → WP 4 → **All waypoints reached – calling mission/complete**
8. Mission state transitions to **COMPLETE (5)**

Total run time: ~60–75 s.

---

## 6. Monitor topics

```bash
# Hazard level: 0=clear 1=caution 2=blocked
ros2 topic echo /obstacles/hazard_level

# Ultrasonic ranges (6 sensors; NaN = nothing in range)
ros2 topic echo /ultrasonic/ranges

# Safety stop flag
ros2 topic echo /safety/stop

# Velocity sent to Gazebo
ros2 topic echo /cmd_vel_gz

# Mission state: 0=idle 1=armed 2=running 3=paused 4=aborted 5=complete
ros2 topic echo /mission/state
```

---

## 7. Abort / reset / rerun

```bash
# Stop a running mission
ros2 service call /mission/abort std_srvs/srv/Trigger {}

# Return to IDLE (from PAUSED, ABORTED, or COMPLETE)
ros2 service call /mission/reset std_srvs/srv/Trigger {}

# Re-arm and start
ros2 service call /mission/arm   std_srvs/srv/Trigger {}
ros2 service call /mission/start std_srvs/srv/Trigger {}
```

---

## Summary

| Terminal | Role |
|----------|------|
| 1 | Gazebo + bridge + `sim_helpers_node` (world, robot model, `/odom/raw`, heartbeat, `/cmd_vel_gz`) |
| 2 | Bringup (safety_manager, ultrasonic_guard, mission_manager, waypoint_follower, localization) |
| 3 | E-stop release (one-shot) |
| 4 | Arm + start mission |

### Obstacle sensing in WSL2

`sim_helpers_node` subscribes to `/odom/raw` and uses ray-circle intersection against the
`obstacle_positions` parameter (default: `[5.0, 0.0, 0.36]` — obstacle at x=5, y=0, radius 0.36 m)
to synthesise the same `/ultrasonic/ranges` that real hardware produces from scan topics.

To add more obstacles, edit the `obstacle_positions` parameter in `sim_helpers_node.py`
(flat list: x0 y0 r0 x1 y1 r1 …).
