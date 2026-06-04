# 📦 modbus_devices

ROS 2 package untuk komunikasi **Modbus RTU (RS485)** berbasis **Python (rclpy)**.
Dirancang untuk sistem **AMR / AGV** dengan prinsip **1 serial port = 1 node**, aman untuk penggunaan industrial.

Package ini telah diuji untuk membaca data dari **driver motor ZLAC8015D**, meliputi:

* Bus Voltage
* Motor Temperature (Left & Right)
* Driver Temperature
* Status koneksi device

---

## ✨ Fitur Utama

* ✅ Modbus RTU (RS485)
* ✅ Non-blocking (Modbus dijalankan di background thread)
* ✅ Aman untuk ROS executor
* ✅ Auto handle device OFF / no response
* ✅ Data dalam **engineering unit** (Volt, °C)
* ✅ Clean shutdown (Ctrl+C)
* ✅ Konfigurasi fleksibel via parameter ROS

---

## 📁 Struktur Package

```
modbus_devices/
├── modbus_devices/
│   ├── __init__.py
│   ├── modbus_client.py      # Wrapper Modbus RTU (pymodbus)
│   └── example_node.py       # Node pembacaan voltage & temperature
├── launch/
│   └── example.launch.py     # Launch file
├── package.xml
├── setup.py
└── README.md
```

---

## 🔧 Dependency

### ROS 2

* ROS 2 Humble
* Python 3.10

### Python library

```bash
pip3 install pymodbus==3.5.2
```

---

## 🚀 Build & Run

### Build package

```bash
cd ~/amr_ws
colcon build --packages-select modbus_devices
source install/setup.bash
```

### Run dengan launch file

```bash
ros2 launch modbus_devices example.launch.py
```

---

## ⚙️ Parameter Node

Semua parameter dapat diatur melalui launch file atau `--ros-args`.

| Parameter           | Default        | Keterangan                      |
| ------------------- | -------------- | ------------------------------- |
| `port`              | `/dev/ttyUSB0` | Path serial RS485               |
| `baudrate`          | `115200`       | Baudrate Modbus RTU             |
| `slave_id`          | `1`            | Modbus slave ID                 |
| `reg_bus_voltage`   | `0x20A0`       | Register bus voltage (0-based)  |
| `reg_motor_temp`    | `0x20A3`       | Register motor temperature      |
| `reg_driver_temp`   | `0x20AF`       | Register driver temperature     |
| `voltage_scale`     | `0.001`        | Skala raw → Volt                |
| `driver_temp_scale` | `0.1`          | Skala raw → °C                  |
| `poll_period`       | `1.0`          | Interval polling Modbus (detik) |
| `data_timeout`      | `2.0`          | Timeout data valid (detik)      |

📌 **Catatan:** semua alamat register sudah disesuaikan dengan **0-based addressing** (`alamat datasheet - 1`).

---

## 📡 Topic yang Dipublish

### Sensor Data

| Topic                      | Type      | Unit |
| -------------------------- | --------- | ---- |
| `/sensor/voltage`          | `Float32` | Volt |
| `/sensor/motor_temp_left`  | `Float32` | °C   |
| `/sensor/motor_temp_right` | `Float32` | °C   |
| `/sensor/driver_temp`      | `Float32` | °C   |

### Status

| Topic               | Type   | Keterangan                   |
| ------------------- | ------ | ---------------------------- |
| `/sensor/connected` | `Bool` | `true` jika device responsif |

---

## 🔍 Contoh Monitoring

```bash
ros2 topic echo /sensor/voltage
ros2 topic echo /sensor/motor_temp_left
ros2 topic echo /sensor/motor_temp_right
ros2 topic echo /sensor/driver_temp
ros2 topic echo /sensor/connected
```

---

## 🧠 Arsitektur Node

```
Modbus RTU (RS485)
      ↓
Background Thread (I/O)
      ↓
Shared Data Buffer
      ↓
ROS Timer (non-blocking)
      ↓
ROS Topic
```

📌 Semua operasi I/O (Modbus) dijalankan di **thread terpisah**, sehingga **ROS executor tidak pernah blocking**.

---

## ⚠️ Safety & Reliability

* Data lama **tidak dipublish** jika device OFF
* Status koneksi selalu tersedia
* Cocok untuk:

  * Safety interlock
  * Diagnostics
  * Monitoring HMI

---

## 🔧 Contoh Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='modbus_devices',
            executable='example_node',
            name='modbus_example',
            parameters=[
                {
                    'port': '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_xxxx',
                    'baudrate': 115200,
                    'slave_id': 1,
                    'voltage_scale': 0.001
                }
            ]
        )
    ])
```

---

## 🛣️ Roadmap Pengembangan

* [ ] Status Word decode (RUN / ALARM / ESTOP)
* [ ] Error Code mapping
* [ ] `/diagnostics` integration
* [ ] Relay / motor control service
* [ ] YAML-based register map
* [ ] Web / HMI integration

---

## 🧑‍💻 Catatan Engineer

> Jangan publish data lama
> Jangan blocking ROS executor
> Jangan percaya datasheet mentah tanpa verifikasi lapangan

Package ini dibuat mengikuti **best practice ROS 2 untuk sistem industrial**.
