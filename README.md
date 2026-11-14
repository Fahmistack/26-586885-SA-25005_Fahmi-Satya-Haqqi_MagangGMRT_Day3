# Robot 5 Servo dengan ESP32, IMU, dan PIR

## 1. Profile

| | |
|---|---|
| **Nama Lengkap** | `Fahmi Satya Haqqi` |
| **NIM** | `26/586885/SA/25005` |

## 2. Description

Sistem ini mengendalikan **5 servo motor** pada robot berdasarkan data kemiringan dan rotasi (roll, pitch, yaw) dari sensor IMU serta deteksi gerakan eksternal dari sensor PIR. Program berjalan pada mikrokontroler **ESP32** dan disimulasikan di **Wokwi**.

- **Mikrokontroler:** ESP32
- **Sensor:**
  - **MPU6050** (IMU 6-axis: akselerometer + gyro) untuk membaca roll, pitch, dan yaw. Sensor ini dipakai sebagai pengganti BNO055 karena BNO055 tidak tersedia di Wokwi.
  - **PIR (Passive Infrared)** untuk mendeteksi gerakan eksternal.
- **Aktuator:** 5 servo motor (Servo 1 sampai Servo 5).
- **Protokol komunikasi:**
  - **I2C** untuk membaca data IMU (SDA = GPIO 21, SCL = GPIO 22).
  - **PWM 50 Hz** (periode 20 ms) untuk mengontrol servo: −90° = 1000 µs, 0° = 1500 µs, +90° = 2000 µs.

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

## 3. Feature Table

| Fitur | Input (Sensor) | Output (Aktuator) | Perilaku |
|---|---|---|---|
| **Roll** | MPU6050 (rotasi sumbu x) | Servo 1 dan 2 | Roll positif: kedua servo berputar ke arah negatif. Roll negatif: kedua servo berputar ke arah positif (sudut servo = −roll). |
| **Pitch** | MPU6050 (rotasi sumbu y) | Servo 3 dan 4 | Servo berputar searah dengan pitch, baik positif maupun negatif (sudut servo = pitch). |
| **Yaw** | MPU6050 (rotasi sumbu z) | Servo 5 | Servo 5 mengikuti arah rotasi yaw. Setelah yaw berhenti, servo diam 1 detik, lalu kembali ke 0° tanpa bergantung pada masukan sensor. |
| **Gerakan Eksternal** | PIR (pin OUT) | Servo 1 sampai 5 | Saat gerakan terdeteksi, semua servo serentak berputar ke +45°, bertahan 1,5 detik, lalu serentak kembali ke 0°. Sensor IMU diabaikan selama urutan ini. |

Ketentuan umum: posisi awal 0°, rentang rotasi dan rentang servo ±90°.

## 4. How to Run

**Link Wokwi:** `https://wokwi.com/projects/480636265081807873`

**Library yang dibutuhkan**

```
ESP32Servo
Adafruit MPU6050
Adafruit Unified Sensor
Adafruit BusIO
```

**Langkah menjalankan simulasi**

1. Buka link Wokwi di atas, atau buat proyek **ESP32** baru lalu salin isi `robot_imu_servo.ino` ke `sketch.ino`, `diagram.json` ke `diagram.json`, dan `libraries.txt` ke `libraries.txt`.
2. Pastikan baris `#define IMU_SERIAL 0` bernilai 0 (memakai sensor MPU6050).
3. Klik tombol hijau **Start**.
4. Tunggu 1-2 detik sampai kalibrasi gyro selesai. Serial Monitor menampilkan `Siap.`. Jangan ubah slider MPU6050 selama kalibrasi.

**Cara memicu input uji**

| Pengujian | Cara memicu | Hasil yang diharapkan |
|---|---|---|
| **Kasus Uji 1: Roll +30°, Pitch −20°** | Ketik `t` di Serial Monitor lalu Enter. Atau klik MPU6050 dan setel akselerometer ke X ≈ 0,342 g, Y ≈ 0,470 g, Z ≈ 0,814 g. | Servo 1 dan 2 = **1333 µs**, Servo 3 dan 4 = **1389 µs** |
| **Kasus Uji 2: Yaw berhenti** | Klik MPU6050, setel gyro Z ke nilai bukan nol (misalnya 30°/s) selama 1-2 detik, lalu kembalikan ke 0. | Servo 5 berhenti, diam 1 detik, lalu kembali ke 0° (1500 µs). Serial Monitor mencetak `Yaw berhenti: tunda 1 detik` dan `Servo 5 kembali ke 0 deg`. |
| **Gerakan Eksternal (PIR)** | Klik sensor PIR saat simulasi berjalan, lalu pilih **Simulate Motion**. | Semua servo serentak ke +45° (1750 µs), bertahan 1,5 detik, lalu kembali ke 0°. |

**Mode tanpa sensor (opsional):** ubah `#define IMU_SERIAL 1`, lalu beri sudut manual lewat Serial Monitor: `r30` (roll), `p-20` (pitch), `y45` (yaw), `t` (Kasus Uji 1), `z` (nol-kan roll dan pitch), `m` (picu PIR).

Kalau arah roll, pitch, atau yaw terbalik, ubah `SIGN_ROLL`, `SIGN_PITCH`, atau `SIGN_YAW` menjadi `-1.0f`.
