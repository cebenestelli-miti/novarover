#!/usr/bin/env bash
# Quick diagnostics for Gazebo sim: run with bringup in another terminal (or after it failed).
# Usage: source install/setup.bash && ./scripts/diagnose_sim.sh

set -e
echo "=== ROS 2 workspace ==="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo ""

echo "=== Nodes (expect: mission_manager, waypoint_follower, safety_manager, ...) ==="
ros2 node list 2>&1 || true
echo ""

echo "=== Mission services (expect: /mission/arm, /mission/start, ...) ==="
ros2 service list 2>&1 | grep -E "mission|mission_manager" || echo "(none found)"
echo ""

echo "=== Key topics ==="
ros2 topic list 2>&1 | grep -E "odom|cmd_vel|mission/state|safety|ultrasonic|estop" || true
echo ""

echo "=== If bringup is running: mission/state (state 2 = RUNNING) ==="
timeout 2 ros2 topic echo /mission/state --once 2>&1 || echo "(topic not available or no message)"
echo ""

echo "Done. If node list is empty, start bringup in another terminal first."
