# AMR Bringup & SLAM Mapping Guide

Dokumen ini menjelaskan langkah lengkap menjalankan sistem AMR mulai dari
bringup, SLAM mapping menggunakan **slam_toolbox**, visualisasi di **RViz2**,
hingga penyimpanan peta.

---

## 1. Prasyarat

- Ubuntu 22.04
- ROS 2 Humble
- Workspace: `~/amr_ws`

Package yang digunakan:
- amr_motor
- sllidar_ros2
- slam_toolbox
- nav2_map_server
- rviz2

---

## 2. Struktur Workspace

```text
amr_ws/
├── src/
│   ├── amr_motor/
│   │   ├── launch/
│   │   │   └── bringup.launch.py
│   │   ├── config/
│   │   │   └── slam_toolbox.yaml
│   │   └── README.md
│   └── sllidar_ros2/
├── build/
├── install/
└── maps/
```

---

## 3. Build Workspace

```bash
cd ~/amr_ws
colcon build
source install/setup.bash
```

---

## 4. Konfigurasi SLAM Toolbox

File konfigurasi:
```text
amr_motor/config/slam_toolbox.yaml
```

```yaml
slam_toolbox:
  ros__parameters:

    odom_frame: odom
    map_frame: map
    base_frame: base_link

    scan_topic: /scan
    mode: mapping

    throttle_scans: 2
    scan_queue_size: 10
    transform_timeout: 0.5
    tf_buffer_duration: 30.0

    publish_tf: true
    use_scan_matching: true

    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.05

    use_sim_time: false
```

---

## 5. Menjalankan Bringup

```bash
ros2 launch amr_motor bringup.launch.py
```

---

## 6. TF Tree yang Diharapkan

```text
map
 └── odom
      └── base_link
           └── laser
```

---

## 7. Visualisasi di RViz2

```bash
rviz2
```

Fixed Frame: `map`

Tambahkan:
- TF
- LaserScan (`/scan`)
- Map
- Odometry (`/odom`)

---

## 8. Menyimpan Peta

```bash
mkdir -p ~/amr_ws/maps

ros2 run nav2_map_server map_saver_cli \
  -f ~/amr_ws/maps/amr_awg_map
```

---

## 9. Hasil

```text
amr_ws/maps/
├── amr_awg_map.pgm
└── amr_awg_map.yaml
```

---

## 10. Selesai

Mapping selesai dan siap digunakan untuk navigasi (Nav2 + AMCL).
