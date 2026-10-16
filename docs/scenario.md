# Scenario: Greenhouse Row Follower (draft)

## Task

An autonomous robot drives between greenhouse plant rows, keeping a fixed distance to the row,
detects ripe (red) fruit with its camera and stops to mark it, makes a U-turn at the end of the row
and continues along the next row. It stops if something blocks its path.

## Sensors (4 types for a 3-person group)

| Sensor | Role in behavior | ROS 2 topic (planned) | Message type | Visualization |
|--------|------------------|-----------------------|--------------|---------------|
| 2D LiDAR | Distance to the row (P-control), end-of-row detection | `/scan` | `sensor_msgs/msg/LaserScan` | RViz LaserScan |
| Camera (right side) | Ripe (red) fruit detection (HSV mask) | `/camera/image_raw` | `sensor_msgs/msg/Image` | rqt_image_view / RViz Image (+ debug mask image) |
| IMU | Heading (yaw) to finish the 180° arc U-turn, tilt monitoring | `/imu` | `sensor_msgs/msg/Imu` | RViz Imu plugin / rqt_plot |
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
