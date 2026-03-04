# Robot geometry for simulation

Yes — Gazebo and ROS need the robot’s physical details so the sim matches the real mower. Those details live in the **robot model** (SDF/URDF) and in **plugin parameters**.

## Where the numbers live

| What | Where in this project | Current sim value | Update when you have real specs |
|------|------------------------|-------------------|----------------------------------|
| **Wheel separation (wheelbase)** | `src/mower_description/models/mower/model.sdf` → `<wheel_separation>` in the DiffDrive plugin | `0.5` m | Measure left-to-right wheel center distance |
| **Wheel radius** | Same file → `<wheel_radius>` in DiffDrive plugin; also in each wheel `<geometry><cylinder><radius>` | `0.1` m | Measure drive wheel radius |
| **Base size (length × width × height)** | Same file → base_link `<box><size>` (visual + collision) | `0.8 0.5 0.2` m | Measure chassis length (x), width (y), height (z) |
| **Wheel positions (left/right)** | Same file → `<pose>` of `left_wheel` and `right_wheel` links | left: `0 0.25 0.1`, right: `0 -0.25 0.1` | Should match wheel_separation/2 in y |
| **Base mass / inertia** | Same file → base_link `<inertial>` | mass 50 kg, simple inertia | Weigh robot; optionally compute inertia |
| **Ultrasonic sensor positions** | Same file → one sensor per ultrasonic (see below) | Default poses for 0=front left … 5=rear | Measure sensor positions and yaw on robot |
| **Odometry frame** | Same file → DiffDrive plugin `<frame_id>`, `<child_frame_id>` | `odom`, `base_link` | Usually keep as-is |

Convention: **base_link** has **x forward**, **y left**, **z up** (ROS REP-103). So “front” is +x, “left” is +y.

## Ultrasonic positions and index mapping (fixed in software)

Physical layout: **front (left, middle, right)**, **left side**, **right side**, **rear** — 6 sensors total.

The stack expects exactly this **index order** on `/ultrasonic/ranges` (UltrasonicArray):

| Index | Position        | Description              | Direction (yaw in base_link) |
|------:|-----------------|--------------------------|------------------------------|
| 0     | front left      | Front left               | Forward-left (+x and +y)     |
| 1     | front middle    | Front center             | Forward (+x)                 |
| 2     | front right     | Front right              | Forward-right (+x and -y)    |
| 3     | left side       | Left side                | Left (+y)                    |
| 4     | right side      | Right side               | Right (-y)                   |
| 5     | rear            | Rear                     | Back (-x)                    |

Sensor poses in the SDF use this same index order so Gazebo topics can be bridged or aggregated into `UltrasonicArray` (range_m[0] … range_m[5]).  
**Note:** The real robot uses ultrasonic (time-of-flight) sensors. In Gazebo Harmonic there is no dedicated ultrasonic sensor type, so the sim uses a range-finding sensor (narrow beam) as a stand-in for each ultrasonic; behavior (distance to obstacles) is comparable.

## What to do with your real robot numbers

1. **Measure** wheel separation, wheel radius, base length/width/height, and (optionally) the (x, y, z, yaw) of each ultrasonic on the chassis.
2. **Edit** `src/mower_description/models/mower/model.sdf`:
   - DiffDrive: `<wheel_separation>`, `<wheel_radius>`.
   - base_link: `<box><size>`, `<inertial>`.
   - Wheel link poses if needed.
   - Each ultrasonic link `<pose>` (and sensor direction) if you added the 6 ray sensors.
3. **Rebuild** (optional if using symlink install):  
   `colcon build --symlink-install --packages-select mower_description`
4. **Relaunch** Gazebo; the sim will use the updated geometry.

## Ultrasonic sensors in the sim (already in the model)

The mower SDF includes **6 ultrasonic sensors** (simulated as range sensors; one per ultrasonic) with fixed index order 0–5. Each has a `<pose>` (x, y, z, roll, pitch, yaw) in `base_link` and publishes a Gazebo topic `/model/mower/ultrasonic/0` … `/model/mower/ultrasonic/5`. To feed the autonomy stack you still need to turn those into one ROS topic:

- **Option A:** Add a small **aggregator node** that subscribes to the 6 Gazebo range/LaserScan topics (via `ros_gz_bridge`), and publishes `mower_msgs/UltrasonicArray` on `/ultrasonic/ranges` with `range_m[0]` … `range_m[5]` in order.
- **Option B:** Extend the bridge config so the 6 topics are bridged to ROS, then a node merges them into `UltrasonicArray`.

Until that’s in place, the sim still runs: bringup can use the mock or Gazebo for odom/cmd_vel; ultrasonics in sim will only affect behavior once the aggregator (and optional bridge entries) are added.

## Optional: IMU and GPS

If you add an IMU or GPS plugin to the model for EKF/GPS testing, their pose and frame should match where the BNO085 and NEO-M9N are mounted on the robot.
