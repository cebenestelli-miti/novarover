# Gazebo setup (latest: Harmonic with ROS 2 Jazzy)

This project is set up for **Gazebo Harmonic** with **ROS 2 Jazzy** via the **ros_gz** (ROS–Gazebo) bridge. Jazzy’s official Gazebo pairing is Harmonic (LTS).

## 1. Install Gazebo and ros_gz

On Ubuntu 24.04 with ROS 2 Jazzy already installed:

```bash
# Gazebo Harmonic + ros_gz (bridge, sim, etc.)
sudo apt-get update
sudo apt-get install ros-jazzy-ros-gz
```

This pulls in Gazebo Harmonic (vendor packages) and the ROS–Gazebo stack (e.g. `ros_gz_sim`, `ros_gz_bridge`).

Optional: install Gazebo binaries from OSRF for the latest Harmonic fixes:

```bash
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

## 2. Build the workspace

From your ROS 2 workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Run the mower in Gazebo

Launch Gazebo with the mower world and the ROS–Gazebo bridge (so `/cmd_vel` and `/odom/raw` match the mower stack):

```bash
ros2 launch mower_description gazebo.launch.py
```

In another terminal, run the rest of the stack. Use real base with **publish_odom:=false** so the bridge is the only source of `/odom/raw`:

```bash
source install/setup.bash
ros2 launch mower_base bringup.launch.py use_real_base:=true publish_odom:=false
```

Gazebo acts as the “base”: it subscribes to `/cmd_vel` and publishes `/odom/raw` via the bridge, so the safety manager, mission manager, waypoint follower, and localization all see the simulated robot.

## 4. Run Gazebo only (no mower stack)

To start Gazebo and the mower model only (e.g. to drive with keyboard or test the sim):

```bash
ros2 launch mower_description gazebo.launch.py
# In another terminal, send velocity commands:
# ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -1
```

## 5. Useful commands

- **List ros_gz packages:** `apt-cache search ros-jazzy-ros-gz`
- **Start Gazebo GUI only:** `gz sim` (then open a world from the GUI)
- **Gazebo docs (Harmonic):** https://gazebosim.org/docs/harmonic

## 6. Version summary

| Item        | Version / package        |
|------------|---------------------------|
| ROS 2      | Jazzy                     |
| Gazebo     | Harmonic (LTS)            |
| ROS–Gazebo | ros-jazzy-ros-gz (meta)   |
| Sim        | ros_gz_sim                |
| Bridge     | ros_gz_bridge             |
