# Scenario: Greenhouse Row Follower (draft)

## Task

An autonomous robot drives between greenhouse plant rows, keeping a fixed distance to the row,
detects ripe (red) fruit with its camera and stops to mark it, makes a U-turn at the end of the row
and continues along the next row. It stops if something blocks its path.

## Sensors (4 types for a 3-person group)

| Sensor | Role in behavior | ROS 2 topic (planned) | Message type | Visualization |
|--------|------------------|-----------------------|--------------|---------------|
| 2D LiDAR | Distance to the row (P-control), end-of-row detection | `/scan` (10 Hz, 360 samples, 0.20–8 m; idx back=0, right=90, front=180, left=270) | `sensor_msgs/msg/LaserScan` | RViz LaserScan |
| Camera (right side) | Ripe (red) fruit detection (HSV mask) | `/camera/image_raw` (640×480, 15 Hz, HFOV 1.3 rad, frame camera_optical_link) | `sensor_msgs/msg/Image` | rqt_image_view / RViz Image (+ debug mask image) |
| IMU | Heading (yaw) to finish the 180° arc U-turn, tilt monitoring | `/imu` (100 Hz; orientation is relative to the spawn heading → use yaw *change*) | `sensor_msgs/msg/Imu` | RViz Imu plugin / rqt_plot |
| Sonar (range) | Front obstacle → emergency stop | `/sonar/front` | `sensor_msgs/msg/Range` | RViz Range |

Notes:
- Gazebo has no separate sonar sensor; a narrow-beam `gpu_lidar` is bridged as `sensor_msgs/msg/Range`
  (`gz.msgs.LaserScan` → `sensor_msgs/msg/Range` in ros_gz_bridge).
- Odometry from the drive plugin does not count as a sensor type.

## Behavior (state machine, single node)

```
          ┌──────────────────────────────────────────────┐
          ▼                                              │
   ROW_FOLLOW ──(camera: red fruit centered)──▶ FRUIT_STOP (wait N s, log/mark) ─┘
     │   ▲
     │   └──────────(yaw turned 180° [IMU])──────┐
     ├──(LiDAR: row side lost = end of row)──▶ U_TURN
     │
     └──(sonar: range < d_stop, any state)──▶ OBSTACLE_STOP ──(clear)──▶ previous state
```

- Inputs: `/scan`, `/camera/image_raw`, `/imu`, `/sonar/front`
- Output: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Optional: `/fruit_detections` (counter / marker) for the demo and report

## Robot

- **Drive: 4-wheel skid-steer.** Gazebo `gz::sim::systems::DiffDrive` accepts multiple
  `<left_joint>` / `<right_joint>` entries, so all four wheels are driven from `/cmd_vel`.
  Needs wheel friction tuning (skid-steer slips sideways when turning).
- Narrow, long body to fit between rows (dimensions TBD).
- **Camera: side-mounted, facing the plants (right side).** The robot always keeps the
  followed row on its right:
  - LiDAR follows the right-hand row; the camera sees the same row's fruit.
  - At the end of the row the robot drives past the row end and arcs **right** around it
    (radius ≈ half the aisle pitch) until the IMU yaw has changed by 180°. It is then in the
    next aisle with the other face of the same row on its right, so camera side never changes.

```
   R1      R2      R3
   │   ↑   │   ↓   │        ↑ aisle 1: R2 on the right
   │   ↑   │   ↓   │        ↓ aisle 2: R2 (other face) on the right
   │   ↑ ╭─┴─╮ ↓   │        ╭─╮ U-turn: arc right around R2's end
```

- LiDAR on top (360°), sonar at the front, IMU inside the chassis near the center.

### Dimensions (agreed 2026-10-16)

Units: m, kg. Frames follow REP-103 (x forward, y left, z up). Right side = −y.

| Part | Shape / size | Pose (relative to parent) | Mass |
|------|--------------|---------------------------|------|
| `base_footprint` | – (ground projection) | – | – |
| `base_link` | – (at wheel axle height) | z = 0.075 from `base_footprint` | – |
| `chassis` | box 0.55 × 0.25 × 0.15 | z = +0.04 from `base_link` (≈ 4 cm ground clearance) | 8.0 |
| 4 × wheel | cylinder r = 0.075, width 0.05 | x = ±0.18, y = ±0.15, z = 0 | 0.5 each |
| `lidar_link` | cylinder r = 0.04, h = 0.05 on a short mast | x = 0.10, z ≈ 0.27 (≈ 0.35 m above ground) | 0.2 |
| `camera_link` | box 0.03 × 0.08 × 0.03 on a side mast | x = 0.0, y = −0.15, z ≈ 0.33 (≈ 0.40 m above ground), yaw = −90° (looks right) | 0.1 |
| `sonar_link` | box 0.02 × 0.05 × 0.03 | x = 0.285, z ≈ 0.05 (front face) | 0.05 |
| `imu_link` | – | at `base_link` origin | – |

Derived values:
- Total width 0.35 (wheels outside the 0.25 chassis), length 0.55
- Wheel separation 0.30, wheelbase 0.36, wheel radius 0.075
- LiDAR sits at plant height on purpose so the scan hits the row (not above it)

TF tree:

```
base_footprint → base_link ─┬─ chassis
                            ├─ front_left_wheel / front_right_wheel / rear_left_wheel / rear_right_wheel
                            ├─ lidar_link
                            ├─ camera_link → camera_optical_link
                            ├─ sonar_link
                            └─ imu_link
odom → base_footprint   (from DiffDrive odometry)
```

### World layout (agreed values)

| Parameter | Value |
|-----------|-------|
| Rows | 3 parallel planter rows (→ 2 aisles) |
| Row length | 6.0 |
| Row (planter) width | 0.4, plants 0.6–1.0 tall |
| Aisle width | 1.0 → row pitch 1.4 |
| U-turn arc radius | ≈ 0.7 (half the pitch) |
| Headland (free space at row ends) | ≥ 1.5 |
| Fruit | red spheres (ripe) + green spheres (unripe), r ≈ 0.04, at 0.3–0.6 height |
| Target distance robot center → right row face | ≈ 0.45 |

## World (to be built)

- 2–3 parallel rows of plant models (green cylinders/boxes), red and green spheres as fruit
- Optional obstacle (crate) placed in an aisle for the sonar stop demo

## Responsibilities (draft)

| Member | Sensor | Integration / reading / visualization |
|--------|--------|---------------------------------------|
| Member 1 | LiDAR | |
| Member 2 | Camera | |
| Member 3 | IMU | |
| Shared | Sonar (additional sensor) | |
