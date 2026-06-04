# amr_relay

## 📌 Deskripsi

`amr_relay` adalah package **ROS 2 (Python-only)** untuk mengendalikan **Industrial Modbus RTU 8-Channel Relay (RS485)** pada sistem **AMR / AGV**. Package ini dirancang agar **aman, stabil, dan tahan kesalahan input**, serta cocok untuk penggunaan robot nyata.

---

## ✨ Fitur Utama

* Kontrol relay **ON / OFF** via ROS topic
* Publish **status relay** secara periodik
* **Validasi channel** (hanya 0–7)
* Command tidak valid **tidak dikirim ke Modbus**
* **Auto recovery Modbus RTU** saat fault
* **CSV logging** semua perintah
* **Safety OFF** saat node dihentikan (`Ctrl+C`)
* Python-only (`setup.py`), **tanpa `.srv` dan tanpa CMake**

---

## 🧩 Arsitektur ROS

### Topic yang Digunakan

| Topic           | Tipe                       | Arah      | Keterangan         |
| --------------- | -------------------------- | --------- | ------------------ |
| `/relay/cmd`    | `std_msgs/String`          | Subscribe | Perintah ON / OFF  |
| `/relay/ack`    | `std_msgs/String`          | Publish   | Respon command     |
| `/relay/status` | `std_msgs/Int32MultiArray` | Publish   | Status relay (0/1) |

---

## ▶️ Cara Menjalankan

```bash
cd ~/amr_ws
colcon build --packages-select amr_relay
source install/setup.bash
ros2 run amr_relay relay_node
```

---

## 🎛️ Format Perintah Relay

### Format Benar

```
"<channel> ON"
"<channel> OFF"
```

Contoh:

```bash
ros2 topic pub -1 /relay/cmd std_msgs/String "data: '0 ON'"
ros2 topic pub -1 /relay/cmd std_msgs/String "data: '3 OFF'"
```

> **Catatan:** Gunakan `-1` agar command hanya dikirim **satu kali**.

---

### Channel Tidak Valid

Channel yang diizinkan hanya **0–7**.

Jika user mengirim:

```bash
ros2 topic pub -1 /relay/cmd std_msgs/String "data: '8 ON'"
```

Maka sistem akan:

* Menolak perintah di level aplikasi
* Tidak mengirim apa pun ke Modbus
* Mengirim ACK:

```
ERROR Invalid channel 8, valid 0-7
```

---

## 📥 Melihat Respon (ACK)

Buka terminal terpisah:

```bash
ros2 topic echo /relay/ack
```

Contoh output:

```
data: OK
```

atau

```
data: ERROR Invalid channel 8, valid 0-7
```

---

## 📊 Status Relay

Status relay dipublish **setiap 1 detik** ke:

```bash
ros2 topic echo /relay/status
```

Contoh:

```
data: [1, 0, 0, 1, 0, 0, 0, 0]
```

Artinya:

* Relay 0 dan 3 = ON
* Relay lainnya = OFF

---

## ⚙️ Parameter ROS

| Parameter  | Default        | Keterangan           |
| ---------- | -------------- | -------------------- |
| `port`     | `/dev/ttyUSB0` | Port RS485           |
| `baudrate` | `9600`         | Baudrate Modbus      |
| `slave_id` | `1`            | Modbus slave address |

Contoh penggunaan:

```bash
ros2 run amr_relay relay_node --ros-args -p port:=/dev/ttyUSB1 -p slave_id:=2
```

---

## 📝 Logging CSV

Semua perintah relay dicatat ke file CSV.

📁 Lokasi:

```
~/amr_ws/src/amr_relay/logs/
```

📄 Contoh isi:

```
time,channel,state,result
2026-01-01T10:00:00,0,ON,OK
2026-01-01T10:01:00,8,OFF,INVALID_CHANNEL
```

---

## 🔐 Safety & Fault Handling

### Safety Shutdown

Saat node dihentikan (`Ctrl + C`):

* Node akan mencoba mematikan **semua relay**
* Log:

```
SAFETY OFF → ALL RELAYS OFF
```

> ⚠️ **Catatan penting:**
> Safety berbasis software adalah **best-effort**.
> Untuk sistem industri wajib ditambah **hardware E-Stop / NC relay**.

---

### Modbus Fault & Recovery

Jika terjadi error Modbus (timeout / CRC / write failed):

* Polling status dihentikan
* Node mencoba reconnect otomatis (±5 detik)
* Setelah normal, sistem lanjut otomatis

Kesalahan **input user** (misalnya channel 8) **tidak dianggap fault Modbus**.

---

## ✅ Best Practice

* Gunakan `ros2 topic pub -1` untuk command
* Jangan mengirim channel di luar 0–7
* Jangan spam command terlalu cepat
* Gunakan `/relay/status` untuk monitoring
* Tambahkan **hardware safety** untuk AMR nyata

---

## ⚠️ Keterbatasan

* Modbus RTU tidak fault-tolerant
* Software tidak bisa menjamin relay OFF jika bus RS485 mati total
* Sistem safety berstandar tinggi **harus berbasis hardware**

---

## 🚀 Pengembangan Lanjutan

* Heartbeat watchdog
* State machine relay
* Web UI (FastAPI)
* MQTT / SCADA bridge
* Diagram wiring fail-safe

---

## 🏁 Status

Package ini **stabil**, **siap dipakai**, dan **aman dari kesalahan user umum** untuk aplikasi **AMR / AGV**.
