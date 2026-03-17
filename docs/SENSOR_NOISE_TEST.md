# Sensor noise simulation test

Verify that mower navigation remains stable when obstacle sensor readings are noisy or intermittent.

## Goal

- **Realistic noise:** Gaussian range noise, occasional false obstacle readings, and short dropouts on ultrasonic data.
- **Simulation layer only:** Noise is injected in the mock base; core navigation and ultrasonic_guard logic are unchanged.
- **Normal mission:** Robot runs the same mission (random_field_20m) and should complete it.

## Test expectations

- Robot does **not** stop or oscillate excessively.
- Occasional noise does **not** cause permanent FOLLOW_OBSTACLE state.
- Robot continues progressing through waypoints (WP_REACHED advances).
- Mission completes successfully (“All waypoints reached – calling mission/complete”).

## What was implemented

- **Noise injection** in `mock_base_interface_node.py` when `sensor_noise_test:=true`:
  - **Gaussian noise:** Add N(0, `ultrasonic_noise_std_m`) to each range, clamped to [min, max].
  - **False obstacle:** Each tick, with probability `ultrasonic_false_obstacle_prob`, one front sensor (0/1/2) reports `ultrasonic_false_obstacle_range_m` (e.g. 0.4 m).
  - **Dropouts:** With probability `ultrasonic_dropout_prob`, the next 1–`ultrasonic_dropout_max_ticks` ultrasonic messages are skipped (brief “no data”; guard allows motion).
- **Dedicated launch** `rviz_mission_sensor_noise.launch.py`: RViz + mock base, no virtual obstacles, noise test enabled by default.

## Files changed

| File | Change |
|------|--------|
| `src/mower_base/mower_base/mock_base_interface_node.py` | Added `sensor_noise_test`, noise/dropout params, `_apply_sensor_noise()`, dropout skip in timer. |
| `src/mower_bringup/launch/rviz_mission_sensor_noise.launch.py` | **New.** Launch for sensor noise test (mission + mock base + safety + guard + mission stack + RViz). |
| `docs/SENSOR_NOISE_TEST.md` | **New.** This file. |

## Commands to run the test

**1. Build and source**

```bash
cd /home/corye/ros2_ws
colcon build
source install/setup.bash
```

**2. Terminal 1 – launch (leave running)**

```bash
cd /home/corye/ros2_ws
source install/setup.bash

ros2 launch mower_bringup rviz_mission_sensor_noise.launch.py
```

**3. Terminal 2 – release E-stop and start mission**

```bash
cd /home/corye/ros2_ws
source install/setup.bash

ros2 topic pub /mower/estop mower_msgs/msg/EStop "{engaged: false}" -1
ros2 service call /mission/arm std_srvs/srv/Trigger "{}"
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
ros2 service call /mission/resume std_srvs/srv/Trigger "{}"
```

Optional: override noise strength (e.g. heavier test):

```bash
ros2 launch mower_bringup rviz_mission_sensor_noise.launch.py \
  ultrasonic_false_obstacle_prob:=0.05 \
  ultrasonic_dropout_prob:=0.02
```

## Log messages that indicate correct behavior

1. **Noise test active (startup)**  
   From `mock_base_interface_node`:  
   `Sensor noise test ON: noise_std=0.050 false_obstacle_prob=0.020 dropout_prob=0.010`

2. **Waypoint progression**  
   From `waypoint_follower_node`:  
   `WP_REACHED idx=1/8`, `WP_REACHED idx=2/8`, … up to the last waypoint. Index should increase over time without long stalls.

3. **Mission completion**  
   From `waypoint_follower_node`:  
   `All waypoints reached – calling mission/complete`

4. **No permanent FOLLOW_OBSTACLE**  
   You may see brief BUG/FOLLOW_OBSTACLE or CAUTION when a false obstacle fires; the robot should return to GO_TO_GOAL and keep advancing. If you see “Skipping unreachable waypoint” repeatedly or the robot stuck for tens of seconds, that may indicate noise is too aggressive or a logic bug.

5. **Dropouts (optional)**  
   `ultrasonic_guard_node` may log at DEBUG: “ultrasonic no recent data (timeout), allowing motion” during a dropout; that is expected and correct (motion allowed when data is missing).

## Pass criteria

- Mission completes with “All waypoints reached”.
- WP_REACHED count equals number of waypoints (e.g. 8 for random_field_20m).
- Robot does not remain stopped or in FOLLOW_OBSTACLE for the majority of the run.
- No (or very few) “Skipping unreachable waypoint” for this no-obstacle scenario.

## Tuning noise (optional)

| Parameter | Default | Effect |
|-----------|--------|--------|
| `ultrasonic_noise_std_m` | 0.05 | Larger = noisier ranges. |
| `ultrasonic_false_obstacle_prob` | 0.02 | Higher = more frequent false short-range readings. |
| `ultrasonic_false_obstacle_range_m` | 0.4 | Reported range when false obstacle triggers. |
| `ultrasonic_dropout_prob` | 0.01 | Higher = more frequent short dropouts. |
| `ultrasonic_dropout_max_ticks` | 2 | Max consecutive ticks without publishing. |

To disable noise and run the same mission with clean sensors, use the random-obstacles launch without obstacles, or run this launch with `sensor_noise_test:=false` (and ensure the mock base supports that; it does).
