# Greenhouse Row Follower

BIMU4101 – Introduction to Mobile Robotics, Midterm Project

> **Greenhouse row follower:** a 4-wheel skid-steer robot that drives between greenhouse plant rows,
> keeps a fixed distance to the row, detects ripe (red) fruit and makes U-turns at the row ends.
> Design details: [docs/scenario.md](docs/scenario.md)

Custom mobile robot modeled in URDF/Xacro, simulated in Gazebo and controlled through ROS 2.

## Environment

| Component | Version |
|-----------|---------|
| OS        | Ubuntu 26.04 LTS |
| ROS 2     | Lyrical |
| Gazebo    | Jetty |

## Group Members

| Name | Student ID | GitHub |
|------|------------|--------|
| TODO | TODO       | TODO   |
| TODO | TODO       | TODO   |
| TODO | TODO       | TODO   |

## Repository Layout

```
src/
├── midterm_description/   # URDF/Xacro robot model, meshes, RViz config
├── midterm_bringup/       # Single launch file, Gazebo worlds, ros_gz bridge config
└── midterm_behavior/      # Sensor-based behavior node (Python)
docs/                      # Report notes, screenshots, demo video links
```

Quick command reference (build, run, test, troubleshooting): [COMMANDS.md](COMMANDS.md)

## Installation

```bash
sudo apt install ros-lyrical-ros-gz ros-lyrical-xacro ros-lyrical-robot-state-publisher \
                 ros-lyrical-rviz2 ros-lyrical-teleop-twist-keyboard

git clone https://github.com/msenduran/Greenhouse-row-follower.git ~/greenhouse_ws   # the repo is the workspace
cd ~/greenhouse_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
# Greenhouse world + robot spawn + bridge + RViz (single launch file)
ros2 launch midterm_bringup sim.launch.py

# Drive manually with WASD (second terminal): hold w/a/s/d, q/e = forward arcs, PgUp/PgDn = speed
ros2 run midterm_behavior wasd_teleop
# or the standard teleop (i/j/k/l keys)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# or
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"

# Fruit detector (started by sim.launch.py; debug view):
ros2 run rqt_image_view rqt_image_view /fruit_detector/debug_image

# Autonomous behavior: start it together with the simulation ...
ros2 launch midterm_bringup sim.launch.py behavior:=true
# ... or separately, once the simulation is running
ros2 run midterm_behavior row_follower --ros-args -p use_sim_time:=true
ros2 topic echo /behavior/state
```

Launch arguments: `behavior:=true` (start the row follower), `rviz:=false`, `headless:=true` (no Gazebo GUI), `x:=` / `y:=` / `yaw:=` (spawn pose), `world:=<path to .sdf>`.

## Nodes

| Node | Package | Subscribes | Publishes |
|------|---------|------------|-----------|
| `fruit_detector` | midterm_behavior | `/camera/image_raw` | `/fruit_detector/fruit` (`geometry_msgs/PointStamped`: x = horizontal offset −1..1, y = vertical offset, z = area ratio; only when a ripe fruit is seen), `/fruit_detector/debug_image` |
| `row_follower` | midterm_behavior | `/scan`, `/imu`, `/fruit_detector/fruit`, `/odom` | `/cmd_vel`, `/behavior/state` (`std_msgs/String`), `/behavior/markers` (RViz: state text, marked fruit) |
| `wasd_teleop` | midterm_behavior | – | `/cmd_vel` |

Unit tests: `colcon test --packages-select midterm_behavior` (fruit detection on synthetic images, row line fitting).

## Sensors

| Sensor type | Topic | Message type | Visualization | Responsible |
|-------------|-------|--------------|---------------|-------------|
| 2D LiDAR    | `/scan` | `sensor_msgs/msg/LaserScan` | RViz (LaserScan) | TODO |
| IMU         | `/imu`  | `sensor_msgs/msg/Imu`       | RViz (rviz_imu_plugin), rqt_plot | TODO |
| Camera      | `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/msg/Image`, `CameraInfo` | RViz (Image), rqt_image_view | TODO |
| TODO (4th sensor type) | TODO | TODO | TODO | TODO |
