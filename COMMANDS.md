# Komutlar

Projeyi derlemek, çalıştırmak, test etmek ve hata ayıklamak için sık kullanılan komutlar.
Tüm komutlar workspace kökünden çalıştırılır:

```bash
cd "/home/samsapulee/Desktop/School/Mobile Robotics/MidtermProject"
```

> Klasör yolunda boşluk var (`Mobile Robotics`): yolu her zaman tırnak içinde yaz.

## 1. Derleme

```bash
source /opt/ros/lyrical/setup.bash              # ROS 2 (bashrc'de yoksa)
colcon build --symlink-install                  # tüm paketler
colcon build --symlink-install --packages-select midterm_behavior   # tek paket
source install/setup.bash                       # her yeni terminalde bir kez
```

- `--symlink-install` ile Python, launch, xacro, world ve yaml değişiklikleri **yeniden derlemeden** geçerli olur.
  Yeni dosya, yeni node (`setup.py` entry point) veya `CMakeLists.txt` / `package.xml` değişikliğinde tekrar derle.
- `Package 'midterm_bringup' not found` → o terminalde `source install/setup.bash` yapılmamış.
- Temiz derleme: `rm -rf build install log && colcon build --symlink-install`

## 2. Simülasyon

```bash
ros2 launch midterm_bringup sim.launch.py                    # Gazebo + robot + bridge + fruit_detector + RViz
ros2 launch midterm_bringup sim.launch.py behavior:=true     # + otonom row_follower
```

| Argüman | Varsayılan | Açıklama |
|---------|------------|----------|
| `behavior` | `false` | `row_follower` node'unu da başlat |
| `rviz` | `true` | RViz'i aç |
| `headless` | `false` | Gazebo GUI'siz (sadece sunucu) |
| `x`, `y`, `yaw` | `-0.5`, `0.65`, `0.0` | Robotun başlangıç konumu (1. koridor, orta sıra sağda) |
| `world` | `greenhouse.sdf` | Dünya dosyası |

## 3. Robotu sürmek / davranış

```bash
ros2 run midterm_behavior wasd_teleop          # w/a/s/d basılı tut, q/e yay, PgUp/PgDn hız, boşluk dur
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # alternatif (i/j/k/l)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"                 # dur

# Simülasyon açıkken davranışı ayrı başlatmak
ros2 run midterm_behavior row_follower --ros-args -p use_sim_time:=true
ros2 run midterm_behavior row_follower --ros-args -p use_sim_time:=true -p rows_to_cover:=0 -p forward_speed:=0.3
ros2 topic echo /behavior/state
```

`row_follower` parametreleri: `forward_speed`, `target_distance`, `kp_distance`, `kp_angle`, `max_angular`,
`turn_radius`, `turn_speed`, `turn_advance`, `rows_to_cover` (0 = sonsuz), `fruit_center_tolerance`,
`fruit_stop_time`, `obstacle_distance`, `obstacle_half_angle`.

## 4. Sensör verisini görmek

```bash
ros2 topic list
ros2 topic hz /scan                                   # LiDAR 10 Hz
ros2 topic echo /scan --once
ros2 topic echo /imu --once                           # IMU 100 Hz
ros2 topic echo /fruit_detector/fruit                 # sadece meyve görülürken yayınlanır
ros2 topic info /camera/image_raw --verbose           # tip, publisher/subscriber, QoS
ros2 interface show sensor_msgs/msg/LaserScan

ros2 run rqt_image_view rqt_image_view /camera/image_raw
ros2 run rqt_image_view rqt_image_view /fruit_detector/debug_image
ros2 run rqt_plot rqt_plot /imu/angular_velocity/z /imu/linear_acceleration/x
```

LiDAR indeksleri (360 örnek, `angle_min = -π`): arka = 0, sağ = 90, ön = 180, sol = 270.

## 5. Rapor için görseller

```bash
ros2 run rqt_graph rqt_graph                          # node/topic grafiği
ros2 run tf2_tools view_frames                        # TF ağacı → frames_*.pdf (gitignore'da)
ros2 run rqt_tf_tree rqt_tf_tree --force-discover     # canlı TF ağacı
ros2 run tf2_ros tf2_echo base_footprint lidar_link   # iki frame arası dönüşüm
ros2 node info /row_follower
```

## 6. Testler

```bash
# Birim testleri (meyve tespiti, sıra çizgisi uydurma)
cd src/midterm_behavior && python3 -m pytest -q test/ && cd ../..
# veya colcon ile
colcon test --packages-select midterm_behavior && colcon test-result --verbose

# Xacro / URDF kontrolü
xacro src/midterm_description/urdf/greenhouse_bot.urdf.xacro > /tmp/bot.urdf && check_urdf /tmp/bot.urdf
gz sdf -k src/midterm_bringup/worlds/greenhouse.sdf   # dünya dosyası geçerli mi
```

### Arayüzsüz simülasyon testi

Açık bir simülasyonla karışmaması için farklı bir `ROS_DOMAIN_ID` kullan:

```bash
export ROS_DOMAIN_ID=42
ros2 launch midterm_bringup sim.launch.py headless:=true rviz:=false behavior:=true
gz model -m greenhouse_bot -p        # Gazebo'daki gerçek konum (ground truth): x y z / roll pitch yaw
gz model --list
gz topic -l                           # Gazebo tarafındaki topic'ler (bridge'den önce)
gz topic -e -t /scan -n 1             # Gazebo tarafında tek mesaj
```

## 7. Sorun giderme

```bash
# Gazebo kapandı ama sunucu arkada kaldıysa (eski dünya açılıyor, simülasyon yavaş)
pgrep -af gz
pkill -f gz-sim-main
```

> **Claude için not:** Bash aracından `pkill -f <desen>` çalıştırırken desen komut satırının kendisiyle
> eşleşir ve aracın kendi kabuğunu öldürür. Köşeli parantez kullan: `pkill -f '[g]z-sim-main'`,
> `pgrep -af '[r]ow_follower'`.

- Gazebo log'larındaki `gz_frame_id ... not defined in SDF` uyarıları zararsız (frame id'ler doğru geliyor).
- `libEGL warning: ... dri2 screen` uyarıları headless çalışmada normal.
- Robot hareket etmiyorsa: `ros2 topic info /cmd_vel` ile publisher var mı, `gz topic -e -t /cmd_vel` ile Gazebo'ya ulaşıyor mu kontrol et.

## 8. Git

```bash
git status
git pull --rebase
git add -A && git commit -m "Kısa açıklama"
git push
git log --oneline -10
```

- Repo: https://github.com/msenduran/Greenhouse-row-follower
- Commit mesajlarına Claude imzası (`Co-Authored-By`) eklenmez.
- `build/`, `install/`, `log/`, `frames*.pdf` gitignore'da.
