// Nama : Fahmi Satya Haqqi
// NIM  : 26/586885/SA/25005

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const int   SERVO_PIN[5] = {32, 33, 25, 26, 27};   // Servo 1..5
const int   PIN_PIR      = 34;

const int   PWM_MIN_US   = 1000;                   // -90 deg
const int   PWM_MID_US   = 1500;                   //   0 deg
const int   PWM_MAX_US   = 2000;                   // +90 deg

const float EXT_TARGET_DEG = 45.0f;                // target semua servo saat PIR
const unsigned long EXT_HOLD_MS   = 1500;          // lama bertahan di target
const unsigned long EXT_SETTLE_MS = 1000;          // waktu kembali ke 0

const float YAW_RATE_THR = 3.0f;                   // deg/s: di bawah ini = berhenti
const unsigned long YAW_STOP_MS   = 200;           // harus stabil selama ini
const unsigned long YAW_HOLD_MS   = 1000;          // diam 1 detik (sesuai soal)
const unsigned long YAW_SETTLE_MS = 400;           // waktu servo 5 kembali ke 0
const float YAW_DEADBAND = 1.0f;                   // deg

const float TC1_ROLL = 30.0f, TC1_PITCH = -20.0f;  // Kasus Uji 1
const float TC1_TOL  = 2.0f;                       // toleransi deteksi (deg)
const unsigned long TC1_STABLE_MS = 300;           // nilai harus stabil selama ini

const float SIGN_ROLL = 1.0f, SIGN_PITCH = 1.0f, SIGN_YAW = 1.0f;
const float ALPHA = 0.98f;                         // bobot gyro di complementary filter
const unsigned long LOOP_MS = 10;                  // periode loop (~100 Hz), non-blocking

Adafruit_MPU6050 mpu;
Servo servo[5];
int   pwmNow[5]      = {1500, 1500, 1500, 1500, 1500};
int   lastPrinted[5] = {0, 0, 0, 0, 0};

enum YawState { YAW_FOLLOW, YAW_HOLD, YAW_RETURN };
enum ExtState { EXT_IDLE, EXT_GO, EXT_RETURN };
YawState yawState = YAW_FOLLOW;
ExtState extState = EXT_IDLE;

float fRoll = 0, fPitch = 0, fYaw = 0;             // sensor
float gxBias = 0, gyBias = 0, gzBias = 0;          // bias gyro
unsigned long imuPrevUs = 0;

bool  ovRoll = false, ovPitch = false, ovYaw = false;
float vRoll = 0, vPitch = 0, vYaw = 0;

float yawRef = 0, prevYaw = 0, s5Angle = 0;
unsigned long prevMs = 0, lastYawMove = 0, yawT = 0, extT = 0, lastPrintMs = 0;
unsigned long lastTick = 0;                        // pewaktu loop non-blocking
bool pirPrev = false, pirForce = false;
bool tc1Shown = false;

float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

int angleToUs(float deg) {
  deg = constrain(deg, -90.0f, 90.0f);
  return (int)lroundf(PWM_MID_US + deg * ((PWM_MAX_US - PWM_MID_US) / 90.0f));
}

void setServo(int i, float deg) {
  int us = angleToUs(deg);
  pwmNow[i] = us;
  servo[i].writeMicroseconds(us);
}

void setAll(float deg) {                           // semua servo serentak
  for (int i = 0; i < 5; i++) setServo(i, deg);
}

void accelAngles(const sensors_event_t &a, float &roll, float &pitch) {
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  roll  = atan2f(ay, az) * RAD_TO_DEG;
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

void readIMU(float &roll, float &pitch, float &yaw) {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long nowUs = micros();
  float dt = (nowUs - imuPrevUs) / 1e6f;
  imuPrevUs = nowUs;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;

  float gx = (g.gyro.x - gxBias) * RAD_TO_DEG;     // deg/s
  float gy = (g.gyro.y - gyBias) * RAD_TO_DEG;
  float gz = (g.gyro.z - gzBias) * RAD_TO_DEG;
  if (fabsf(gz) < 0.5f) gz = 0;                    // redam drift yaw

  float accRoll, accPitch;
  accelAngles(a, accRoll, accPitch);

  fRoll  = ALPHA * (fRoll  + gx * dt) + (1.0f - ALPHA) * accRoll;
  fPitch = ALPHA * (fPitch + gy * dt) + (1.0f - ALPHA) * accPitch;
  fYaw   = wrap180(fYaw + gz * dt);

  roll  = SIGN_ROLL  * fRoll;
  pitch = SIGN_PITCH * fPitch;
  yaw   = SIGN_YAW   * fYaw;

  if (ovRoll)  roll  = vRoll;
  if (ovPitch) pitch = vPitch;
  if (ovYaw)   yaw   = vYaw;
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (!s.length()) return;
  char  c = tolower(s[0]);
  float v = s.substring(1).toFloat();
  switch (c) {
    case 't': ovRoll = ovPitch = true; vRoll = TC1_ROLL; vPitch = TC1_PITCH;
              tc1Shown = false;                    // izinkan blok Kasus Uji 1 tercetak lagi
              break;                               // (dicetak setelah stabil 300 ms)
    case 'r': ovRoll  = true; vRoll  = constrain(v, -90.0f, 90.0f); break;
    case 'p': ovPitch = true; vPitch = constrain(v, -90.0f, 90.0f); break;
    case 'y': ovYaw   = true; vYaw   = constrain(v, -90.0f, 90.0f); break;
    case 'm': pirForce = true; break;
    case 'z':                                      // kembali ke sensor
      ovRoll = ovPitch = ovYaw = false;
      yawRef = SIGN_YAW * fYaw; yawState = YAW_FOLLOW; lastYawMove = millis();
      Serial.println("Paksaan dihapus, membaca sensor MPU6050");
      break;
  }
}

void checkTestCase1(float roll, float pitch) {
  static unsigned long hitSince = 0;
  bool hit = fabsf(roll - TC1_ROLL) <= TC1_TOL && fabsf(pitch - TC1_PITCH) <= TC1_TOL;
  if (!hit) { hitSince = 0; tc1Shown = false; return; }
  if (hitSince == 0) hitSince = millis();
  if (!tc1Shown && millis() - hitSince >= TC1_STABLE_MS) {
    tc1Shown = true;
    Serial.println("=========== KASUS UJI 1 ===========");
    Serial.printf("Terdeteksi: Roll = %.1f deg, Pitch = %.1f deg\n", roll, pitch);
    Serial.printf("Servo 1 PWM = %d us\n", pwmNow[0]);
    Serial.printf("Servo 2 PWM = %d us\n", pwmNow[1]);
    Serial.printf("Servo 3 PWM = %d us\n", pwmNow[2]);
    Serial.printf("Servo 4 PWM = %d us\n", pwmNow[3]);
    Serial.println("===================================");
  }
}

void printStatus(float roll, float pitch, float yaw) {
  bool changed = false;
  for (int i = 0; i < 5; i++) if (pwmNow[i] != lastPrinted[i]) changed = true;
  if (!changed || millis() - lastPrintMs < 200) return;
  lastPrintMs = millis();
  for (int i = 0; i < 5; i++) lastPrinted[i] = pwmNow[i];
  Serial.printf("Roll=%.1f Pitch=%.1f Yaw=%.1f | S1=%d S2=%d S3=%d S4=%d S5=%d us\n",
                roll, pitch, yaw, pwmNow[0], pwmNow[1], pwmNow[2], pwmNow[3], pwmNow[4]);
}

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (int i = 0; i < 5; i++) {
    servo[i].setPeriodHertz(50);                   // periode 20 ms
    servo[i].attach(SERVO_PIN[i], PWM_MIN_US, PWM_MAX_US);
  }
  setAll(0);                                       // posisi awal 0 deg
  pinMode(PIN_PIR, INPUT);

  Wire.begin(21, 22);                              // I2C: SDA=21, SCL=22
  Wire.setClock(400000);
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 tidak terdeteksi, cek wiring I2C!");
    while (true) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  Serial.println("Kalibrasi gyro, jangan ubah slider MPU6050 ...");
  const int N = 200;
  float sx = 0, sy = 0, sz = 0, ar = 0, ap = 0;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z;
    float r, p; accelAngles(a, r, p);
    ar += r; ap += p;
    delay(5);
  }
  gxBias = sx / N; gyBias = sy / N; gzBias = sz / N;
  fRoll = ar / N;  fPitch = ap / N;  fYaw = 0;
  imuPrevUs = micros();

  Serial.println("Siap. Geser slider MPU6050, atau ketik: t, r<n>, p<n>, y<n>, z, m");
  float r, p, y;
  readIMU(r, p, y);
  yawRef = y; prevYaw = y; prevMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTick < LOOP_MS) return;            // tanpa delay()
  lastTick = now;
  handleSerial();

  float roll, pitch, yaw;
  readIMU(roll, pitch, yaw);

  float dt = (now - prevMs) / 1000.0f;
  float yawRate = (dt > 0) ? wrap180(yaw - prevYaw) / dt : 0;
  prevYaw = yaw; prevMs = now;

  bool pirNow = digitalRead(PIN_PIR) || pirForce;
  pirForce = false;
  if (pirNow && !pirPrev && extState == EXT_IDLE) {
    extState = EXT_GO; extT = now;
    setAll(EXT_TARGET_DEG);
    Serial.println("PIR: gerakan eksternal -> semua servo ke target");
  }
  pirPrev = pirNow;

  if (extState != EXT_IDLE) {
    if (extState == EXT_GO && now - extT >= EXT_HOLD_MS) {
      setAll(0); extState = EXT_RETURN; extT = now;
    } else if (extState == EXT_RETURN && now - extT >= EXT_SETTLE_MS) {
      extState = EXT_IDLE;
      yawRef = yaw; yawState = YAW_FOLLOW; lastYawMove = now;
      s5Angle = 0;
    }
    printStatus(roll, pitch, yaw);
    return;
  }

  setServo(0, -roll);
  setServo(1, -roll);
  
  setServo(2, pitch);
  setServo(3, pitch);

  checkTestCase1(roll, pitch);

  switch (yawState) {
    case YAW_FOLLOW:                               // ikut rotasi yaw
      s5Angle = constrain(wrap180(yaw - yawRef), -90.0f, 90.0f);
      setServo(4, s5Angle);
      if (fabsf(yawRate) > YAW_RATE_THR) lastYawMove = now;
      if (fabsf(s5Angle) > YAW_DEADBAND && now - lastYawMove >= YAW_STOP_MS) {
        yawState = YAW_HOLD; yawT = now;
        Serial.printf("[%lu ms] KASUS UJI 2: yaw berhenti, Servo 5 diam di %d us, tunda 1 detik\n",
                      now, pwmNow[4]);
      }
      break;
    case YAW_HOLD:                                 // diam 1 detik, sensor diabaikan
      if (now - yawT >= YAW_HOLD_MS) {
        unsigned long waited = now - yawT;         // lama tunda sebenarnya
        setServo(4, 0); yawState = YAW_RETURN; yawT = now;
        Serial.printf("[%lu ms] KASUS UJI 2: Servo 5 kembali ke 0 deg, PWM = %d us (tunda %lu ms)\n",
                      now, pwmNow[4], waited);
      }
      break;
    case YAW_RETURN:                               // beri waktu servo, sensor diabaikan
      if (now - yawT >= YAW_SETTLE_MS) {
        yawRef = yaw; yawState = YAW_FOLLOW; lastYawMove = now;
      }
      break;
  }

  printStatus(roll, pitch, yaw);
}
