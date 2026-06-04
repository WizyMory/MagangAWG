# 🦾 AMR Motor Node (ZLAC8015D)

ROS 2 motor driver node untuk **AMR / AGV differential drive** berbasis **ZLAC8015D**, menggunakan **Modbus RTU**, dengan fokus pada **stabilitas robot fisik**.

Node ini menggunakan **acceleration & deceleration internal driver ZLAC** (bukan ramp di ROS), sehingga pergerakan lebih halus, minim hentakan, dan responsif.

---

## ✨ Fitur Utama

* Kontrol kecepatan via `/cmd_vel`
* **Acceleration & deceleration ditangani oleh driver ZLAC**
* **STOP instant** (tanpa ROS ramp / deadband)
* Encoder-based **odometry (`/odom`)**
* Publish TF **`odom → base_link`**
* Service **enable / disable motor**
* Service **reset odometry**
* Diagnostic status (tegangan & error code driver)
* Arsitektur decoupled (control loop ≠ Modbus write)

---

## 📡 Topic & Service

### Subscribed Topics

| Topic      | Type                  | Keterangan                          |
| ---------- | --------------------- | ----------------------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Perintah kecepatan linear & angular |

---

### Published Topics

| Topic   | Type                | Keterangan                    |
| ------- | ------------------- | ----------------------------- |
| `/odom` | `nav_msgs/Odometry` | Odometry encoder (diff-drive) |
| `/tf`   | TF                  | Transform `odom → base_link`  |

---

### Services

| Service                     | Type               | Fungsi                 |
| --------------------------- | ------------------ | ---------------------- |
| `/amr_motor/enable`         | `std_srvs/SetBool` | Enable / disable motor |
| `/amr_motor/reset_odometry` | `std_srvs/Trigger` | Reset posisi odometry  |

---

## ▶️ Cara Menjalankan

### 1. Jalankan node

```bash
ros2 run amr_motor amr_motor_node
```

---

### 2. Reset odometry (WAJIB setelah node hidup)

```bash
ros2 service call /amr_motor/reset_odometry std_srvs/srv/Trigger
```

---

### 3. Enable motor

```bash
ros2 service call /amr_motor/enable std_srvs/srv/SetBool "{data: true}"
```

---

### 4. Kontrol robot (contoh keyboard)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

### 5. Disable motor (safety)

```bash
ros2 service call /amr_motor/enable std_srvs/srv/SetBool "{data: false}"
```

---

## ⚙️ Parameter

### Hardware & Kinematics

| Parameter      | Default                 | Keterangan                    |
| -------------- | ----------------------- | ----------------------------- |
| `port`         | `/dev/serial/by-id/...` | Port RS485                    |
| `baudrate`     | `115200`                | Baudrate Modbus               |
| `slave_id`     | `1`                     | ID Modbus ZLAC                |
| `wheel_radius` | `0.065`                 | Radius roda (meter)           |
| `wheel_base`   | `0.3118`                | Jarak antar roda (meter)      |
| `encoder_cpr`  | `4096`                  | Encoder counts per revolution |
| `gear_ratio`   | `1.0`                   | Gear ratio encoder            |

---

### Motion & Safety

| Parameter                  | Default | Keterangan              |
| -------------------------- | ------- | ----------------------- |
| `cmd_vel_timeout`          | `0.2`   | Timeout cmd_vel (detik) |
| `max_rpm`                  | `3000`  | Batas RPM motor         |
| `battery_critical_voltage` | `20.0`  | Cutoff voltage (V)      |

---

### ZLAC Acceleration Profile

| Parameter       | Default | Keterangan               |
| --------------- | ------- | ------------------------ |
| `accel_ms`      | `1000`  | Acceleration time (ms)   |
| `decel_ms`      | `800`   | Deceleration time (ms)   |
| `quick_stop_ms` | `10`    | Emergency stop time (ms) |

> ℹ️ Tidak ada ramp di ROS. Semua akselerasi ditangani langsung oleh driver ZLAC.

---

## 🧭 Odometry & Frame

* Frame standar ROS:

```
odom → base_link
```

* Konvensi arah:

  * +X → depan robot
  * +Y → kiri robot
  * +Z → atas

* Encoder kiri dibalik tanda agar konsisten dengan arah motor.

---

## 🧪 Debug & Validasi

### Cek odometry

```bash
ros2 topic echo /odom
```

### Cek TF

```bash
ros2 run tf2_tools view_frames
```

### Cek diagnostics

```bash
ros2 topic echo /diagnostics
```

---

## ⚠️ Catatan Penting

* ❗ Jangan enable motor sebelum reset odometry
* ❗ Hindari Ctrl+Z (gunakan Ctrl+C)
* ✔ Reset encoder driver **tidak diperlukan**
* ✔ Reset odometry cukup via service ROS

---

## 🚀 Next Step

Node ini siap dikembangkan lebih lanjut:

* EKF (encoder + IMU)
* Nav2 (autonomous navigation)
* Lifecycle node & safety hardening

---

## 👤 Author

AMR / AGV ROS 2 Development
Driver-based acceleration, production-ready architecture

port lidar
/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0 