# BIMU4101 – Mobile Robotics Midterm Project

> TODO: robot name and one-line task description

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
sudo apt install ros-lyrical-ros-gz ros-lyrical-xacro ros-lyrical-robot-state-publisher
# TODO: add other dependencies

git clone <REPO_URL> ~/midterm_ws
cd ~/midterm_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
# TODO: world + robot spawn + RViz in a single launch file
ros2 launch midterm_bringup <TODO>.launch.py

# TODO: behavior node
ros2 run midterm_behavior <TODO>
```

## Sensors

| Sensor type | Topic | Message type | Visualization | Responsible |
|-------------|-------|--------------|---------------|-------------|
| TODO        | TODO  | TODO         | TODO          | TODO        |
