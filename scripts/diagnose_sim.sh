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

echo "=== Mission state (state 2 = RUNNING; if not 2, run arm+start in Terminal 4) ==="
timeout 2 ros2 topic echo /mission/state --once 2>&1 || echo "(topic not available or no message)"
echo ""

echo "=== /safety/stop (must be false to move; if true, run E-stop release in Terminal 3) ==="
timeout 2 ros2 topic echo /safety/stop --once 2>&1 || echo "(topic not available)"
echo ""

echo "=== /odom (must be publishing; if timeout, Gazebo/bridge/localization not running) ==="
timeout 2 ros2 topic echo /odom --once 2>&1 | head -5 || echo "(no message)"
echo ""

echo "=== /cmd_vel (should show non-zero linear.x when mission RUNNING and not blocked) ==="
timeout 2 ros2 topic echo /cmd_vel --once 2>&1 | head -8 || echo "(no message)"
echo ""

echo "Done. If mission state != 2: run arm+start. If safety/stop true: run E-stop. If no /odom: start Gazebo and bringup."
