# Localization config and sensors

Requires **robot_localization**: `sudo apt install ros-jazzy-robot-localization`

## Modes

| Launch | Behavior |
|--------|----------|
| (default) | Simple node: `/odom/raw` → `/odom` + TF |
| `use_ekf:=true` | EKF fuses `/odom/raw` + `/imu/data` (BNO085). Output: `/odom` + TF |
| `use_ekf:=true use_gps:=true` | EKF fuses `/odom/raw` + `/imu/data` + `/odometry/gps`. NavSat converts `/gps/fix` → `/odometry/gps`. Requires NEO-M9N + BNO085. |

**Commands**

```bash
# No fusion
ros2 launch mower_base bringup.launch.py

# IMU fusion (BNO085)
ros2 launch mower_base bringup.launch.py use_ekf:=true

# IMU + GPS fusion (BNO085 + NEO-M9N)
ros2 launch mower_base bringup.launch.py use_ekf:=true use_gps:=true
```

**Config files**

- `ekf.yaml` – EKF with odom + IMU only.
- `ekf_gps.yaml` – EKF with odom + IMU + `/odometry/gps`.
- `navsat_transform.yaml` – NavSat: `/gps/fix` + `/imu/data` + `/odom` → `/odometry/gps`. Set `magnetic_declination_radians` for your location.

## Expected sensors and topics

### NEO-M9N (GNSS)

- **Publish:** `/gps/fix` (NavSatFix), `/gps/status` (GpsStatus for safety_manager).
- With `use_gps:=true`, `navsat_transform_node` converts `/gps/fix` to local-frame `/odometry/gps` for the EKF.

### BNO085 (BNO080) – 9-DOF IMU

- **Publish:**  
  - **`/imu/data`** – `sensor_msgs/msg/Imu`  
    - Orientation (quaternion), angular velocity, linear acceleration.  
    - Frame: **`base_link`** (or consistent with your robot).  
    - Conform to REP-103 (ENU); if the IMU does not remove gravity, set `imu0_remove_gravitational_acceleration: true` in the EKF config (already set in `ekf.yaml`).

With these drivers publishing on **`/odom/raw`**, **`/imu/data`**, and (when used) **`/gps/fix`** and **`/gps/status`**, the rest of the mower stack (localization, waypoint follower, safety) will work as designed.
