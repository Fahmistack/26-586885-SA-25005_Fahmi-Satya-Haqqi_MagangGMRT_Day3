# Robot 5 Servo dengan ESP32, IMU, dan PIR

## 1. Profile

| | |
|---|---|
| **Nama Lengkap** | Fahmi Satya Haqqi |
| **NIM** | 26/586885/SA/25005 |

## 2. Description

Sistem ini mengendalikan **5 servo motor** pada robot berdasarkan data kemiringan dan rotasi (roll, pitch, yaw) dari sensor IMU serta deteksi gerakan eksternal dari sensor PIR. Program berjalan pada mikrokontroler **ESP32** dan disimulasikan di **Wokwi**.

- **Mikrokontroler:** ESP32
- **Sensor:**
  - **MPU6050** (IMU 6-axis: akselerometer + gyro) untuk membaca roll, pitch, dan yaw. Dipakai sebagai pengganti BNO055 karena BNO055 tidak tersedia di Wokwi.
  - **PIR (Passive Infrared)** untuk mendeteksi gerakan eksternal.
- **Aktuator:** 5 servo motor (Servo 1 sampai Servo 5).
- **Protokol komunikasi:**
  - **I2C** untuk membaca data IMU (SDA = GPIO 21, SCL = GPIO 22).
  - **PWM 50 Hz** (periode 20 ms) untuk mengontrol servo: −90° = 1000 µs, 0° = 1500 µs, +90° = 2000 µs.
- **Pewaktuan:** seluruh jeda memakai **state machine berbasis `millis()`**, tanpa `delay()` di dalam `loop()`. Pembacaan sensor dan respons Roll/Pitch tidak terblokir saat Servo 5 menunggu 1 detik.

**Pin yang digunakan**

| Komponen | Pin ESP32 |
|---|---|
| Servo 1 (Roll) | GPIO 32 |
| Servo 2 (Roll) | GPIO 33 |
| Servo 3 (Pitch) | GPIO 25 |
| Servo 4 (Pitch) | GPIO 26 |
| Servo 5 (Yaw) | GPIO 27 |
| MPU6050 SDA / SCL | GPIO 21 / GPIO 22 |
| PIR OUT | GPIO 34 |

Servo 1-2 dan Servo 3-4 dipasang berdekatan, dan tidak ada servo yang memakai jalur I2C.

**State machine**

| State | Fungsi | Transisi |
|---|---|---|
| `YAW_FOLLOW` | Servo 5 mengikuti yaw | Ke `YAW_HOLD` saat yaw berhenti (kecepatan < 3°/s selama 200 ms) |
| `YAW_HOLD` | Servo 5 diam, masukan yaw diabaikan | Ke `YAW_RETURN` setelah 1000 ms |
| `YAW_RETURN` | Servo 5 kembali ke 0° | Ke `YAW_FOLLOW` setelah 400 ms |
| `EXT_IDLE` | Operasi normal | Ke `EXT_GO` saat PIR terpicu |
| `EXT_GO` | Semua servo ke +45°, ditahan 1,5 detik | Ke `EXT_RETURN` |
| `EXT_RETURN` | Semua servo kembali ke 0° | Ke `EXT_IDLE` setelah 1000 ms |

Penjelasan lengkap ada di komentar awal `robot_imu_servo.ino`.

**Asumsi**

- Saat start, sensor diam dan datar (roll = pitch = yaw = 0). Posisi awal servo 0°.
- Yaw dihitung dari integrasi gyro Z karena MPU6050 tidak punya magnetometer.
- Posisi target gerakan eksternal (PIR) adalah +45° untuk semua servo.

**Diagram**

![Diagram blok](diagram_blok.jpg)

![Flowchart](flowchart.jpg)

## 3. Feature Table

| Fitur | Input (Sensor) | Output (Aktuator) | Perilaku |
|---|---|---|---|
| **Roll** | MPU6050 (rotasi sumbu x) | Servo 1 dan 2 | Roll positif: kedua servo berputar ke arah negatif. Roll negatif: kedua servo berputar ke arah positif (sudut servo = −roll). |
| **Pitch** | MPU6050 (rotasi sumbu y) | Servo 3 dan 4 | Servo berputar searah dengan pitch, baik positif maupun negatif (sudut servo = pitch). |
| **Yaw** | MPU6050 (rotasi sumbu z) | Servo 5 | Servo 5 mengikuti arah rotasi yaw. Setelah yaw berhenti, servo diam 1 detik, lalu kembali ke 0° tanpa bergantung pada masukan sensor. |
| **Gerakan Eksternal** | PIR (pin OUT) | Servo 1 sampai 5 | Saat gerakan terdeteksi, semua servo serentak berputar ke +45°, bertahan 1,5 detik, lalu serentak kembali ke 0°. Sensor IMU diabaikan selama urutan ini. |

Ketentuan umum: posisi awal 0°, rentang rotasi dan rentang servo ±90°.

## 4. How to Run

**Link Wokwi:** https://wokwi.com/projects/480636265081807873

**Library yang dibutuhkan** (isi tab `libraries.txt` di Wokwi)

```
ESP32Servo
Adafruit MPU6050
Adafruit Unified Sensor
Adafruit BusIO
```

**Langkah menjalankan simulasi**

1. Buka link Wokwi di atas, atau buat proyek **ESP32** baru lalu salin isi `robot_imu_servo.ino` ke `sketch.ino`, `diagram.json` ke `diagram.json`, dan `libraries.txt` ke `libraries.txt`.
2. Pastikan semua slider MPU6050 di nilai awal: Acceleration X = 0, Y = 0, Z = 1 g, dan Rotation X, Y, Z = 0°/sec.
3. Klik tombol hijau **Start**.
4. Tunggu sampai Serial Monitor menampilkan `Siap.`. Jangan ubah slider MPU6050 selama kalibrasi gyro (sekitar 1 detik).

**Cara memicu input uji**

| Pengujian | Cara memicu | Hasil yang diharapkan |
|---|---|---|
| **Kasus Uji 1: Roll +30°, Pitch −20°** | Ketik `t` di Serial Monitor lalu Enter. Atau klik MPU6050 lalu geser Acceleration Y ke 0,55 g dan X ke 0,40 g (Z tetap 1 g). | Blok `KASUS UJI 1` tercetak. Dengan `t`: Servo 1 dan 2 = **1333 µs**, Servo 3 dan 4 = **1389 µs**. Dengan slider nilainya mendekati (±2°). |
| **Kasus Uji 2: Yaw berhenti** | Klik MPU6050, geser Rotation Z ke sekitar 50°/sec selama 1-2 detik, lalu kembalikan ke 0. Atau ketik `y45` lalu Enter. | Servo 5 berhenti, diam 1 detik, lalu kembali ke 0° (1500 µs). Dua baris `KASUS UJI 2` tercetak dengan selisih waktu 1000 ms. |
| **Gerakan Eksternal (PIR)** | Klik sensor PIR saat simulasi berjalan, lalu pilih **Simulate Motion**. | Semua servo serentak ke +45° (1750 µs), bertahan 1,5 detik, lalu kembali ke 0°. |

**Perintah Serial Monitor** (selalu aktif, diakhiri Enter)

| Perintah | Fungsi |
|---|---|
| `t` | Paksa roll +30° dan pitch −20° (Kasus Uji 1) |
| `r<n>` / `p<n>` / `y<n>` | Paksa roll / pitch / yaw ke n derajat (contoh `r30`, `p-20`, `y45`) |
| `m` | Simulasikan pemicu PIR |
| `z` | Hapus semua paksaan dan kembali membaca sensor MPU6050 |

Setelah memakai `t`, `r`, `p`, atau `y`, ketik `z` agar slider sensor kembali mengendalikan servo.
