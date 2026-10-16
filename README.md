# BIMU4101 – Mobile Robotics Midterm Project

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

## Installation

```bash
sudo apt install ros-lyrical-ros-gz ros-lyrical-xacro ros-lyrical-robot-state-publisher \
                 ros-lyrical-rviz2 ros-lyrical-teleop-twist-keyboard

git clone <REPO_URL> ~/midterm_ws
cd ~/midterm_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
# Greenhouse world + robot spawn + bridge + RViz (single launch file)
ros2 launch midterm_bringup sim.launch.py

# Drive manually with WASD (second terminal): hold w/a/s/d, q/e = forward arcs, +/- = speed
ros2 run midterm_behavior wasd_teleop
# or the standard teleop (i/j/k/l keys)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# or
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"

# TODO: behavior node
ros2 run midterm_behavior <TODO>
```

Launch arguments: `rviz:=false`, `headless:=true` (no Gazebo GUI), `x:=` / `y:=` / `yaw:=` (spawn pose), `world:=<path to .sdf>`.

## Sensors

| Sensor type | Topic | Message type | Visualization | Responsible |
|-------------|-------|--------------|---------------|-------------|
| TODO        | TODO  | TODO         | TODO          | TODO        |
