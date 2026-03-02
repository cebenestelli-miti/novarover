#!/usr/bin/env bash
# Heartbeat timeout test: no background jobs.
# TERMINAL 1 MUST run: ros2 launch mower_base bringup_safety_test.launch.py
# (NOT full bringup - full bringup has mock which publishes heartbeat, so timeout never triggers.)
set -e
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash

echo "=== Ensure Terminal 1 is running: ros2 launch mower_base bringup_safety_test.launch.py ==="
echo "=== (Not full bringup.) Waiting 2s for safety_manager to be up... ==="
sleep 2

echo "=== Feeding ultrasonics for 4s (then stops) ==="
timeout 4 ros2 topic pub -r 10 /ultrasonic/ranges mower_msgs/msg/UltrasonicArray "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, range_m: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0], min_range_m: 0.02, max_range_m: 4.0}" || true

echo "=== Releasing E-stop (no heartbeat yet) ==="
ros2 topic pub --once /mower/estop mower_msgs/msg/EStop "{stamp: {sec: 0, nanosec: 0}, engaged: false, source: 'test'}"

echo "=== Waiting 2s for heartbeat timeout ==="
sleep 2

echo "=== Expect safety/stop true, reason base_heartbeat_timeout ==="
ros2 topic echo /safety/stop --once
ros2 topic echo /safety/state --once

echo "=== Feeding heartbeat for 3s (then stops) ==="
timeout 3 ros2 topic pub -r 10 /base/heartbeat std_msgs/msg/Empty "{}" || true

echo "=== Waiting 1s for resume ==="
sleep 1

echo "=== Expect safety/stop false, state RUNNING ==="
ros2 topic echo /safety/stop --once
ros2 topic echo /safety/state --once

echo "=== Heartbeat timeout test script done ==="
