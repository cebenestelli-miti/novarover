# Localization config and sensors

## EKF fusion (use_ekf:=true)

Requires **robot_localization**:

```bash
sudo apt install ros-jazzy-robot-localization
```

Then:

```bash
ros2 launch mower_base bringup.launch.py use_ekf:=true
```

The EKF fuses:

- **`/odom/raw`** – wheel odometry from the base (mock or real).
- **`/imu/data`** – IMU (orientation, angular velocity, linear acceleration).

Output: **`/odom`** and TF **odom → base_link**.

## Expected sensors and topics

When you add real hardware, these are the topic contracts the stack expects.

### NEO-M9N (GNSS)

- **Publish:**  
  - **`/gps/fix`** – `sensor_msgs/msg/NavSatFix`  
  - **`/gps/status`** – `mower_msgs/msg/GpsStatus` (for safety_manager GPS loss/stale).

Optional later: use **robot_localization** `navsat_transform_node` to feed GPS into the EKF as a second position source (map frame).

### BNO085 (BNO080) – 9-DOF IMU

- **Publish:**  
  - **`/imu/data`** – `sensor_msgs/msg/Imu`  
    - Orientation (quaternion), angular velocity, linear acceleration.  
    - Frame: **`base_link`** (or consistent with your robot).  
    - Conform to REP-103 (ENU); if the IMU does not remove gravity, set `imu0_remove_gravitational_acceleration: true` in the EKF config (already set in `ekf.yaml`).

With these drivers publishing on **`/odom/raw`**, **`/imu/data`**, and (when used) **`/gps/fix`** and **`/gps/status`**, the rest of the mower stack (localization, waypoint follower, safety) will work as designed.
