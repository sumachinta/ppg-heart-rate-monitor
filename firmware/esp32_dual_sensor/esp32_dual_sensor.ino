/*
 * ─────────────────────────────────────────────────────────
 *  ESP32 Feather HUZZAH32 — Dual Sensor Monitor
 *  MAX30102 (Heart Rate + SpO2) + MPU-6050 (Accel + Gyro)
 *
 *  Libraries required (install via Arduino Library Manager):
 *    1. "SparkFun MAX3010x Pulse and Proximity Sensor Library"
 *       by SparkFun Electronics
 *    2. "Adafruit MPU6050"
 *       by Adafruit  (also installs Adafruit Unified Sensor)
 *
 *  Wiring (I2C shared bus):
 *    HUZZAH32 3V  → MAX30102 VIN,  MPU-6050 VCC
 *    HUZZAH32 GND → MAX30102 GND,  MPU-6050 GND
 *    HUZZAH32 SDA (GPIO 23) → MAX30102 SDA, MPU-6050 SDA
 *    HUZZAH32 SCL (GPIO 21/22) → MAX30102 SCL, MPU-6050 SCL
 *    HUZZAH32 GPIO 15 → MAX30102 INT  (optional)
 *
 *  Serial Monitor  : 115200 baud — human-readable labeled output
 *  Serial Plotter  : same output is plotter-compatible
 *                    (comma-separated, consistent column order)
 * ─────────────────────────────────────────────────────────
 */

#include <Wire.h>
#include "MAX30105.h"          // SparkFun MAX3010x library
#include "heartRate.h"         // SparkFun beat-detection helper
#include "spo2_algorithm.h"    // SparkFun SpO2 algorithm
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── I2C pins for HUZZAH32 ──────────────────────────────────
#define SDA_PIN 21
#define SCL_PIN 22

// ── MAX30102 sample buffer (SpO2 algorithm needs 100 samples) ──
#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];

// __ WiFi _____
const char* WIFI_SSID     = "NETGEAR48-5G";
const char* WIFI_PASSWORD = "instantregret";

// ── SpO2 / HR results ──────────────────────────────────────
int32_t  spo2Value       = 0;
int8_t   spo2Valid       = 0;
int32_t  heartRateValue  = 0;
int8_t   hrValid         = 0;

// ── Beat-detection (finger-on detection) ──────────────────
const byte RATE_SIZE = 4;
byte       rates[RATE_SIZE];
byte       rateSpot = 0;
long       lastBeat = 0;
float      beatsPerMinute = 0;
int        beatAvg        = 0;

// ── Objects ───────────────────────────────────────────────
MAX30105     particleSensor;
Adafruit_MPU6050 mpu;

// ── Timing ────────────────────────────────────────────────
unsigned long lastPrintMs    = 0;
const uint16_t PRINT_INTERVAL = 500;   // ms between Serial prints
unsigned long lastSpO2Ms     = 0;
const uint16_t SPO2_INTERVAL = 4000;   // ms between full SpO2 recalc

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("=== ESP32 Dual Sensor Monitor ==="));
  Serial.println(F("MAX30102 (HR + SpO2) + MPU-6050 (Accel + Gyro)"));
  Serial.println();

  // ── Init I2C ──────────────────────────────────────────
  Wire.begin(SDA_PIN, SCL_PIN);

  // ── Init MAX30102 ─────────────────────────────────────
  Serial.print(F("Initializing MAX30102... "));
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("FAILED. Check wiring!"));
    while (true) { delay(1000); }
  }
  Serial.println(F("OK"));

  // Sensor config: optimized for SpO2 + HR
  byte ledBrightness = 60;   // 0=Off, 255=50mA
  byte sampleAverage = 4;    // 1, 2, 4, 8, 16, 32
  byte ledMode       = 2;    // 1=Red only, 2=Red+IR (needed for SpO2)
  int  sampleRate    = 100;  // 50, 100, 200, 400, 800, 1000, 1600, 3200
  int  pulseWidth    = 411;  // 69, 118, 215, 411
  int  adcRange      = 4096; // 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x0A);  // low red LED for proximity check
  particleSensor.setPulseAmplitudeGreen(0);   // green off (not needed)

  // ── Init MPU-6050 ─────────────────────────────────────
  Serial.print(F("Initializing MPU-6050... "));
  if (!mpu.begin()) {
    Serial.println(F("FAILED. Check wiring!"));
    while (true) { delay(1000); }
  }
  Serial.println(F("OK"));

  // Ranges — adjust if signal saturates or is too noisy
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println();
  Serial.println(F("Place finger firmly on MAX30102 sensor."));
  Serial.println(F("Waiting for finger detection..."));
  Serial.println();

  // ── Fill SpO2 buffer with initial readings ─────────────
  fillSpO2Buffer();

  Serial.println(F("--- Streaming started ---"));
  Serial.println();

  // Print plotter header (Arduino Serial Plotter uses first line as labels)
  Serial.println(F("HR_BPM,SpO2_%,AccX_g,AccY_g,AccZ_g,GyrX_ds,GyrY_ds,GyrZ_ds"));
}

// ─────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── Continuous beat detection (runs every loop) ────────
  detectHeartBeat();

  // ── Periodic SpO2 recalculation ────────────────────────
  if (now - lastSpO2Ms >= SPO2_INTERVAL) {
    recalcSpO2();
    lastSpO2Ms = now;
  }

  // ── Print at PRINT_INTERVAL ────────────────────────────
  if (now - lastPrintMs >= PRINT_INTERVAL) {
    lastPrintMs = now;

    // Read MPU-6050
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;
    float gx = gyro.gyro.x;
    float gy = gyro.gyro.y;
    float gz = gyro.gyro.z;

    // ── Human-readable Serial Monitor output ──────────────
    Serial.println(F("─────────────────────────────────────────"));

    // Finger detection (IR > 50000 = finger present)
    long irValue = particleSensor.getIR();
    if (irValue < 50000) {
      Serial.println(F("  ⚠  No finger detected on MAX30102"));
    } else {
      Serial.print  (F("  Heart Rate : "));
      if (hrValid) {
        Serial.print(heartRateValue);
        Serial.println(F(" BPM  ✓"));
      } else {
        Serial.print(beatAvg);
        Serial.println(F(" BPM  (stabilizing...)"));
      }

      Serial.print  (F("  SpO2       : "));
      if (spo2Valid) {
        Serial.print(spo2Value);
        Serial.println(F(" %  ✓"));
      } else {
        Serial.println(F("  -- %  (keep finger still)"));
      }
    }

    Serial.println();
    Serial.print  (F("  Accel  X: ")); Serial.print(ax, 3); Serial.print(F(" g  "));
    Serial.print  (F("Y: "));          Serial.print(ay, 3); Serial.print(F(" g  "));
    Serial.print  (F("Z: "));          Serial.print(az, 3); Serial.println(F(" g"));

    Serial.print  (F("  Gyro   X: ")); Serial.print(gx, 3); Serial.print(F(" °/s  "));
    Serial.print  (F("Y: "));          Serial.print(gy, 3); Serial.print(F(" °/s  "));
    Serial.print  (F("Z: "));          Serial.print(gz, 3); Serial.println(F(" °/s"));

    Serial.println();

    // ── Plotter-friendly CSV line (same values, no units) ─
    // Format: HR_BPM,SpO2_%,AccX,AccY,AccZ,GyrX,GyrY,GyrZ
    int plotHR   = hrValid   ? (int)heartRateValue : beatAvg;
    int plotSpO2 = spo2Valid ? (int)spo2Value      : 0;

    Serial.print(plotHR);    Serial.print(F(","));
    Serial.print(plotSpO2);  Serial.print(F(","));
    Serial.print(ax, 3);     Serial.print(F(","));
    Serial.print(ay, 3);     Serial.print(F(","));
    Serial.print(az, 3);     Serial.print(F(","));
    Serial.print(gx, 3);     Serial.print(F(","));
    Serial.print(gy, 3);     Serial.print(F(","));
    Serial.println(gz, 3);
  }
}

// ─────────────────────────────────────────────────────────
// Beat detection — call continuously in loop()
// Uses SparkFun's checkForBeat() on the IR channel
// ─────────────────────────────────────────────────────────
void detectHeartBeat() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat   = millis();

    beatsPerMinute = 60.0 / (delta / 1000.0);

    if (beatsPerMinute > 20 && beatsPerMinute < 255) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
}

// ─────────────────────────────────────────────────────────
// Fill buffer — called once at startup to prime SpO2 algo
// ─────────────────────────────────────────────────────────
void fillSpO2Buffer() {
  for (byte i = 0; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available())
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2Value, &spo2Valid,
    &heartRateValue, &hrValid
  );
}

// ─────────────────────────────────────────────────────────
// Recalc SpO2 — called every SPO2_INTERVAL ms
// Shifts buffer forward by 25 samples, appends 25 fresh ones
// ─────────────────────────────────────────────────────────
void recalcSpO2() {
  // Dump first 25 samples, shift remaining 75 to front
  for (byte i = 25; i < BUFFER_LENGTH; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }

  // Collect 25 fresh samples
  for (byte i = 75; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available())
      particleSensor.check();

    detectHeartBeat();  // keep beat detection running while collecting

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2Value, &spo2Valid,
    &heartRateValue, &hrValid
  );
}
