# Research: Similar ROS 2 Projects

Date: 2026-10-16. Goal: see how others build a custom ROS 2 mobile robot (URDF/Xacro + Gazebo + multiple sensors + sensor-based behavior node) before designing ours.

## 1. Reference Projects

| Project | What it has | Useful for us |
|---------|-------------|---------------|
| [bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot) | Diff drive, IMU, depth + stereo camera, 2D LiDAR; Gazebo Harmonic (Jazzy branch); sensors toggled by launch args | Package layout (`urdf/ worlds/ launch/ rviz/ config/`), sensor on/off launch arguments |
| [eddiebot-ros](https://github.com/arashsm79/eddiebot-ros) | New Gazebo (`gz-sim-diff-drive-system`), `gpu_lidar`, `rgbd_camera`, `ros_gz_bridge`, `ros_gz_sim` spawn | Closest to our stack: plugin names + bridge + spawn pattern |
| [DimitrisKatos/my_robot](https://github.com/DimitrisKatos/my_robot) | Xacro diff drive, lidar, camera, simple Python obstacle avoidance | Step-by-step xacro + simple behavior node (README in Greek) |
| [oomwoo-one](https://github.com/makerspet/oomwoo-one) | Robot-vacuum xacro: round body, LiDAR turret, 2 wheels + caster, EKF fusing `/odom` + `/imu` | Example of an original-looking body with a clear task |
| [differential-drive-robot-model-ros-gazebo](https://github.com/EnriqManComp/differential-drive-robot-model-ros-gazebo) | Bumper/contact sensors on wheels and chassis (ROS 1) | Contact sensor setup; collision names need `_collision` suffix |
| [likerobotics/ros2_course](https://github.com/likerobotics/ros2_course) | Course on Jazzy + Harmonic: xacro robot, TF in RViz, camera/lidar simulation | Similar to our course assignment |
| [prathapselvakumar/ROS-2](https://github.com/prathapselvakumar/ROS-2) | MSc coursework: simulated diff drive robot, Gazebo bridges, Nav2 | Shows how graded coursework repos are organized |
| [LinkForge diff drive tutorial](https://linkforge.readthedocs.io/en/latest/tutorials/building_diff_drive.html) | Diff drive + `gpu_lidar` for modern Gazebo, ros2_control | Alternative modeling workflow |
| [ROS 2 Gazebo tutorial](https://docs.ros.org/en/iron/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html) | Official diff drive + lidar bridged to ROS 2, viewed in RViz | `parameter_bridge` usage |

## 2. Behavior Node Examples

| Behavior | Reference | Idea |
|----------|-----------|------|
| Obstacle avoidance (LiDAR) | [Hello Robot – collision avoidance](https://docs.hello-robot.com/0.3/ros2/example_3) | `LaserScan` sub → speed scaled by min front distance → `Twist` pub |
| Wall following (LiDAR) | [UMass CS603 WF project](https://hcr.cs.umass.edu/courses/compsci603/projects/Compsci_603_Project2_WF_ROS2.pdf), [The Construct thread](https://get-help.theconstruct.ai/t/wall-following-works-in-gazebo-but-fails-on-real-robot-crashes-into-wall/36182) | P-controller on side distance; verify scan indices (front/left/right) in RViz |
| Color object tracking (camera) | [Husarion OpenCV tracking](https://husarion.com/tutorials/ros2-tutorials/5-opencv-tracking/), [Indro vision module](https://indrorobotics.notion.site/Vision-Module-2fe1bb7999b2804fbab0da42e95fa8e9) | `cv_bridge` → HSV `inRange` mask → centroid error → P-controller on `angular.z` |
| Ball chasing | [MathWorks track & follow](https://au.mathworks.com/help/ros/ug/track-and-follow-an-object.html) | Search (rotate) → approach → keep fixed distance (MATLAB, logic only) |
| Bump-and-go FSM | *A Concise Introduction to Robot Programming with ROS 2* (book), [Fordham WanderT3](https://twiki.ace.fordham.edu/pub/Main/GoogleCSR/WanderT3.py.txt) | States: FORWARD → BACK → TURN; trigger from bumper or lidar cone |
| State machine libs | [YASMIN](https://index.ros.org/r/yasmin), [SMACC2](https://docs.ros.org/en/jazzy/p/sm_dance_bot_strikes_back/) | Optional; a hand-written FSM in one node is enough for us |

**Common pattern:** one node, several sensor subscriptions that only update state variables, and a timer (10–20 Hz) that runs the decision logic (FSM) and publishes `geometry_msgs/msg/Twist` on `/cmd_vel`. Easy to explain in the presentation (inputs → rule → output).

## 3. Technical Notes for Our Stack (Lyrical + Jetty)

- Lyrical ↔ Jetty is the official binary pairing ([ros_gz](https://github.com/gazebosim/ros_gz)). Most online examples target Jazzy + Harmonic; they are mostly portable but check plugin names and parameters.
- In new Gazebo, world/model/sensor plugins are all **System plugins** ([feature comparison](https://gazebosim.org/docs/latest/comparison/)). Use `gz::sim::systems::...`, not the Gazebo Classic `libgazebo_ros_*.so` plugins, which many older tutorials use.
- `ign`/`ignition` names are renamed to `gz` ([migration guide](https://gazebosim.org/docs/latest/migration_from_ignition/)).
- World needs the `Sensors` system (rendering sensors: camera, lidar), `Imu` system and `Contact` system in addition to the robot's `DiffDrive` (and `JointStatePublisher`).

### ros_gz_bridge type mappings ([README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md))

| Data | ROS type | Gazebo type |
|------|----------|-------------|
| LiDAR | `sensor_msgs/msg/LaserScan` | `gz.msgs.LaserScan` |
| 3D LiDAR / depth | `sensor_msgs/msg/PointCloud2` | `gz.msgs.PointCloudPacked` |
| Camera | `sensor_msgs/msg/Image`, `CameraInfo` | `gz.msgs.Image`, `gz.msgs.CameraInfo` |
| IMU | `sensor_msgs/msg/Imu` | `gz.msgs.IMU` |
| Contact / bumper | `ros_gz_interfaces/msg/Contacts` | `gz.msgs.Contacts` |
| Range (sonar/IR) | `sensor_msgs/msg/Range` | `gz.msgs.LaserScan` |
| Magnetometer | `sensor_msgs/msg/MagneticField` | `gz.msgs.Magnetometer` |
| GPS | `sensor_msgs/msg/NavSatFix` | `gz.msgs.NavSat` |
| Cmd vel | `geometry_msgs/msg/Twist` | `gz.msgs.Twist` |
| Odometry | `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` |
| TF | `tf2_msgs/msg/TFMessage` | `gz.msgs.Pose_V` |
| Clock | `rosgraph_msgs/msg/Clock` | `gz.msgs.Clock` |

## 4. Observations

- Most repos are "generic diff-drive robot + lidar + camera"; **few define a concrete task**. A clear scenario will make our project stand out and is required by the assignment.
- Combining behaviors in a small FSM (e.g. explore with LiDAR, recover with bumper, approach target with camera) is a recurring and well-documented pattern, and it lets every member's sensor take part in the behavior.
- Bumper/contact sensors are less documented in new Gazebo than lidar/camera; plan extra time for that one.
- Watch out for LiDAR index ↔ direction mapping and frame orientation; check in RViz first.

## 5. Candidate Task Scenarios (to discuss)

| Scenario | Sensors (4 types) | Behavior |
|----------|-------------------|----------|
| Warehouse patrol robot | LiDAR, camera, IMU, bumper | Patrol aisles avoiding obstacles; stop when a colored "hazard" marker is seen; back up on bump |
| Search & rescue scout | LiDAR, camera, IMU, contact | Explore rubble world; turn toward and approach a colored "victim" object; stop at a safe distance |
| Greenhouse row follower | LiDAR, camera, IMU, sonar/range | Follow the plant row at a fixed distance (wall following); detect ripe (red) fruit with camera |
| Hospital delivery robot | LiDAR, camera, IMU, bumper | Corridor wall following; stop at a colored door marker; safety stop on contact |
