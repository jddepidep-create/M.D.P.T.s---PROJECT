/*
  CanSat telemetry - ESP32-C3 SuperMini (no SD, no magnetometer)
  Sensors (all on one I2C bus):
    BMP280 (0x77 or 0x76) + AHT20 (0x38)  -> pressure, temperature, humidity, altitude
    BMI160 (0x68 or 0x69)                  -> accelerometer + gyroscope

  Wiring: 3V3 -> VCC of all modules, GND -> GND, GPIO8 -> SDA, GPIO9 -> SCL
  Libraries (Library Manager): "Adafruit BMP280 Library", "Adafruit AHTX0"
  Arduino IDE: Board "ESP32C3 Dev Module", USB CDC On Boot = ENABLED

  Output (20 Hz, CSV, 115200 baud):
  D,ms,ax,ay,az,gx,gy,gz,roll,pitch,yaw,tempC,hum,pressHPa,alt,vspeed
  Send 'z' over serial to re-zero altitude and orientation.
*/
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define LED_PIN 10

Adafruit_BMP280 bmp(&Wire);
Adafruit_AHTX0 aht;
uint8_t bmiAddr = 0;
bool haveBmp = false, haveAht = false, haveBmi = false;

// BMI160 config: accel +-16g (2048 LSB/g), gyro +-2000 dps (16.4 LSB/dps)
const float ACC_LSB = 2048.0f, GYR_LSB = 16.4f;

float gbx = 0, gby = 0, gbz = 0;           // gyro bias
float roll = 0, pitch = 0, yaw = 0;        // degrees
float p0 = 1013.25f;                        // ground pressure (hPa)
float temp = 0, hum = 0, press = 0, alt = 0, altS = 0, vs = 0;
float ax = 0, ay = 0, az = 1, gx = 0, gy = 0, gz = 0;

void bmiWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bmiAddr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
bool bmiRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(bmiAddr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(bmiAddr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool bmiInit() {
  for (uint8_t a : {0x68, 0x69}) {
    bmiAddr = a;
    uint8_t id = 0;
    if (bmiRead(0x00, &id, 1) && id == 0xD1) {
      bmiWrite(0x7E, 0xB6); delay(100);   // soft reset
      bmiWrite(0x7E, 0x11); delay(60);    // accel normal mode
      bmiWrite(0x7E, 0x15); delay(120);   // gyro normal mode
      bmiWrite(0x40, 0x29);               // accel 200 Hz
      bmiWrite(0x41, 0x0C);               // accel +-16g
      bmiWrite(0x42, 0x29);               // gyro 200 Hz
      bmiWrite(0x43, 0x00);               // gyro +-2000 dps
      return true;
    }
  }
  return false;
}

bool readBmi() {
  uint8_t b[12];
  if (!bmiRead(0x0C, b, 12)) return false;
  auto s = [&](int i) { return (int16_t)((b[i + 1] << 8) | b[i]); };
  gx = s(0) / GYR_LSB - gbx; gy = s(2) / GYR_LSB - gby; gz = s(4) / GYR_LSB - gbz;
  ax = s(6) / ACC_LSB;       ay = s(8) / ACC_LSB;       az = s(10) / ACC_LSB;
  return true;
}

void calibrateGyro() {            // keep the board still!
  float sx = 0, sy = 0, sz = 0; int n = 0;
  gbx = gby = gbz = 0;
  for (int i = 0; i < 200; i++) {
    if (readBmi()) { sx += gx; sy += gy; sz += gz; n++; }
    delay(5);
  }
  if (n) { gbx = sx / n; gby = sy / n; gbz = sz / n; }
}

void zeroAll() {
  if (haveBmp) {
    float s = 0;
    for (int i = 0; i < 30; i++) { s += bmp.readPressure() / 100.0f; delay(20); }
    p0 = s / 30.0f;
  }
  altS = 0; vs = 0; yaw = 0;
  if (haveBmi) {
    calibrateGyro();
    readBmi();
    roll = atan2f(ay, az) * RAD_TO_DEG;
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  while (!Serial) delay(100);
  Serial.println("\n\n=== CanSat Booting ===");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  haveBmp = bmp.begin(0x77) || bmp.begin(0x76);
  if (haveBmp) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_1);
  }
  haveAht = aht.begin();
  haveBmi = bmiInit();
  delay(100);
  Serial.printf("# BMP280:%d AHT20:%d BMI160:%d\n", haveBmp, haveAht, haveBmi);
  delay(100);
  Serial.println("# Keep still, calibrating...");
  zeroAll();
  delay(100);
  Serial.println("# Ready");
}

uint32_t tImu = 0, tBaro = 0, tAht = 0, tOut = 0, tLed = 0;

void loop() {
  uint32_t us = micros();
  uint32_t now = millis();
  if (Serial.available() && Serial.read() == 'z') zeroAll();

  // ---- IMU @ 200 Hz + complementary filter ----
  if (haveBmi && us - tImu >= 5000) {
    float dt = (us - tImu) * 1e-6f; if (tImu == 0) dt = 0.005f;
    tImu = us;
    if (readBmi()) {
      float accMag = sqrtf(ax * ax + ay * ay + az * az);
      float rollAcc = atan2f(ay, az) * RAD_TO_DEG;
      float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
      roll += gx * dt; pitch += gy * dt; yaw += gz * dt;
      if (accMag > 0.7f && accMag < 1.3f) {
        roll = 0.98f * roll + 0.02f * rollAcc;
        pitch = 0.98f * pitch + 0.02f * pitchAcc;
      }
    }
  }

  // ---- Barometer @ 20 Hz ----
  if (haveBmp && now - tBaro >= 50) {
    float dt = (now - tBaro) * 1e-3f; tBaro = now;
    press = bmp.readPressure() / 100.0f;
    alt = 44330.0f * (1.0f - powf(press / p0, 0.1903f));
    float prev = altS;
    altS = 0.7f * altS + 0.3f * alt;
    vs = 0.85f * vs + 0.15f * ((altS - prev) / dt);
    if (!haveAht) temp = bmp.readTemperature();
  }

  // ---- AHT20 every 2 s ----
  if (haveAht && now - tAht >= 2000) {
    tAht = now;
    sensors_event_t h, t; aht.getEvent(&h, &t);
    hum = h.relative_humidity; temp = t.temperature;
  }

  // ---- LED status indicator ----
  float accelMag = sqrtf(ax * ax + ay * ay + az * az);
  uint32_t ledPeriod = 1000;  // default: slow blink (stationary)
  if (accelMag > 1.5f) {
    ledPeriod = 100;  // fast blink (accelerating)
  } else if (alt > 5.0f) {
    ledPeriod = 300;  // medium blink (in flight)
  }
  if (now - tLed >= ledPeriod) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    tLed = now;
  }

  // ---- Telemetry @ 20 Hz ----
  if (now - tOut >= 50) {
    tOut = now;
    Serial.printf("D,%lu,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\n",
                  now, ax, ay, az, gx, gy, gz, roll, pitch, yaw, temp, hum, press, altS, vs);
  }
}
