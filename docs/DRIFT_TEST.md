# Long mission drift test (mp_test)

Validate that the mower can run a 400+ waypoint mission without accumulating navigation drift or degrading tracking quality over time.

## Setup

- **Mission:** `mp_test_converted.wgs84` (414 waypoints from converted Mission Planner QGC WPL 110).
- **Run:** RViz + mock-base only (no Gazebo). No changes to navigation logic; optional lightweight drift-check logging.

## 1. Build and convert (if needed)

```bash
cd /home/corye/ros2_ws
colcon build
source install/setup.bash
```

If you need to regenerate the converted mission:

```bash
ros2 run mower_mission qgc_mission_converter_node \
  --ros-args \
  -p input_qgc_file:=/home/corye/missions/mp_test.waypoints \
  -p output_wgs84_file:=/home/corye/ros2_ws/src/mower_mission/config/missions/mp_test_converted.wgs84
```

## 2. Run the drift test

**Terminal 1 (leave running):**

```bash
cd /home/corye/ros2_ws
source install/setup.bash

ros2 launch mower_bringup rviz_mission_test.launch.py \
  mission_file:=/home/corye/ros2_ws/src/mower_mission/config/missions/mp_test_converted.wgs84 \
  farm_config:=/home/corye/ros2_ws/src/mower_mission/config/farm_origin.yaml \
  mission_frame_id:=map \
  waypoint_tolerance_m:=0.5 \
  auto_spawn_near_first_wp:=true \
  drift_check_ticks:=200
```

**Terminal 2:**

```bash
cd /home/corye/ros2_ws
source install/setup.bash

ros2 topic pub /mower/estop mower_msgs/msg/EStop "{engaged: false}" -1
```

Then:

```bash
ros2 service call /mission/arm std_srvs/srv/Trigger "{}"
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
ros2 service call /mission/resume std_srvs/srv/Trigger "{}"
```

## 3. How to monitor

### Waypoint progression

- **Logs:** `waypoint_follower_node` prints `WP_REACHED idx=N/414` when a waypoint is reached. Index should increase steadily.
- **Throttled pose:** With `debug_log:=true` (default), look for `WP[n/414]` in the periodic line; `n` should advance over time.
- **Topic (optional):** Waypoint markers are on `/mission/waypoints_markers`; path on `/mission/path`.

### Unexpected skips

- **Logs:** Grep for skips:  
  `Skipping unreachable waypoint`  
  Any occurrence means the follower skipped a waypoint (no-progress timeout or orbit). For a drift test with no obstacles, you expect zero or very few skips.

```bash
# From a saved log or while running (Terminal 2):
ros2 topic echo /rosout | grep -E "Skipping unreachable|WP_REACHED|mission/complete"
```

### Heading error on straight segments (drift-check log)

- **Logs:** With `drift_check_ticks:=200`, every 20 s the node prints:  
  `DRIFT_CHECK wp=N/414 dist=D heading_err_max_rad=E`  
  - `wp` = current waypoint index.  
  - `dist` = distance to current waypoint (m).  
  - `heading_err_max_rad` = max |heading error| (rad) over the last 200 ticks.  
  On straight rows, small `heading_err_max_rad` (e.g. &lt; 0.1–0.2 rad) indicates good tracking.

### Tracking quality later in the mission

- Compare **early** vs **late** `DRIFT_CHECK` lines (e.g. first 50 waypoints vs last 50):
  - If `heading_err_max_rad` grows significantly in the second half, tracking may be degrading (e.g. drift).
  - If it stays similar, tracking is stable over the long mission.

## 4. Review results after the run

Save the launch terminal output to a file, then:

```bash
# Waypoint progression: count WP_REACHED
grep "WP_REACHED" drift_test.log | wc -l

# Skips (should be 0 or very low for open-field mp_test)
grep "Skipping unreachable waypoint" drift_test.log

# Drift-check series (waypoint index vs heading_err_max_rad)
grep "DRIFT_CHECK" drift_test.log

# Mission completion
grep -E "All waypoints reached|mission/complete" drift_test.log
```

**Pass:** Mission reaches completion (`All waypoints reached – calling mission/complete`), no or very few skips, and `heading_err_max_rad` in `DRIFT_CHECK` does not grow systematically toward the end of the mission.

**Fail:** Mission never completes, many skips, or clearly increasing heading error in the last half of the mission.
