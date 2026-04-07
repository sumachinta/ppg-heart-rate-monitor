/*
 * ─────────────────────────────────────────────────────────
 *  ESP32 Feather HUZZAH32 — Dual Sensor Monitor + WiFi Stream
 *  DEBUG VERSION — outputs CSV on Serial for live plotting
 *
 *  Serial format (500ms interval):
 *  DATA,<irRaw>,<redRaw>,<bpm>,<beatAvg>,<spo2>,<spo2Valid>,<hrValid>
 * ─────────────────────────────────────────────────────────
 */

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── WiFi Credentials ─────────────────────────────────────
const char* WIFI_SSID     = "NETGEAR48";
const char* WIFI_PASSWORD = "instantregret";

// ── MQTT Broker ───────────────────────────────────────────
const char* MQTT_BROKER   = "broker.hivemq.com";
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT   = "esp32_sensor_001";

// ── MQTT Topics ───────────────────────────────────────────
const char* TOPIC_HR      = "esp32/heartrate";
const char* TOPIC_SPO2    = "esp32/spo2";
const char* TOPIC_ACCEL   = "esp32/accel";
const char* TOPIC_GYRO    = "esp32/gyro";
const char* TOPIC_STATUS  = "esp32/status";

// ── I2C pins ─────────────────────────────────────────────
#define SDA_PIN 21
#define SCL_PIN 22

// ── MAX30102 buffer ───────────────────────────────────────
#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];

// ── SpO2 / HR results ─────────────────────────────────────
int32_t  spo2Value      = 0;
int8_t   spo2Valid      = 0;
int32_t  heartRateValue = 0;
int8_t   hrValid        = 0;

// ── Beat detection ────────────────────────────────────────
const byte RATE_SIZE      = 4;
byte       rates[RATE_SIZE];
byte       rateSpot       = 0;
long       lastBeat       = 0;
float      beatsPerMinute = 0;
int        beatAvg        = 0;

// ── Objects ───────────────────────────────────────────────
MAX30105         particleSensor;
Adafruit_MPU6050 mpu;
WiFiClient       wifiClient;
PubSubClient     mqttClient(wifiClient);

// ── Timing ────────────────────────────────────────────────
unsigned long lastPrintMs = 0;
unsigned long lastSpO2Ms  = 0;
unsigned long lastMqttMs  = 0;
const uint16_t PRINT_INTERVAL = 50; // 50ms → 20Hz to plotter
const uint16_t SPO2_INTERVAL  = 4000;
const uint16_t MQTT_INTERVAL  = 1000;

// ─────────────────────────────────────────────────────────
void connectWiFi() {
  Serial.print(F("# Connecting to WiFi"));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print(F("# WiFi connected! IP: "));
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  while (!mqttClient.connected()) {
    Serial.print(F("# Connecting to MQTT..."));
    if (mqttClient.connect(MQTT_CLIENT)) {
      Serial.println(F(" connected!"));
      mqttClient.publish(TOPIC_STATUS, "ESP32 online");
    } else {
      Serial.print(F(" failed rc="));
      Serial.print(mqttClient.state());
      Serial.println(F(" retry in 3s"));
      delay(3000);
    }
  }
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("# === ESP32 Debug Mode ==="));

  Wire.begin(SDA_PIN, SCL_PIN);

  // Init MAX30102
  Serial.print(F("# Initializing MAX30102... "));
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("FAILED!"));
    while (true) { delay(1000); }
  }
  Serial.println(F("OK"));

  byte ledBrightness = 60;
  byte sampleAverage = 1; // no averaging, full 100Hz
  byte ledMode       = 2;
  int  sampleRate    = 100;
  int  pulseWidth    = 411;
  int  adcRange      = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  // Init MPU-6050
  Serial.print(F("# Initializing MPU-6050... "));
  if (!mpu.begin()) {
    Serial.println(F("FAILED!"));
    while (true) { delay(1000); }
  }
  Serial.println(F("OK"));
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  connectWiFi();
  connectMQTT();

  Serial.println(F("# Place finger on MAX30102 — filling buffer..."));
  fillSpO2Buffer();

  // CSV header — Python plotter looks for this
  Serial.println(F("# Streaming. Format:"));
  Serial.println(F("# DATA,irRaw,redRaw,bpm,beatAvg,spo2,spo2Valid,hrValid,fingerOn"));
  Serial.println(F("HEADER,irRaw,redRaw,bpm,beatAvg,spo2,spo2Valid,hrValid,fingerOn"));
}

// ─────────────────────────────────────────────────────────
void loop() {
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  unsigned long now = millis();

  detectHeartBeat();

  if (now - lastSpO2Ms >= SPO2_INTERVAL) {
    recalcSpO2();
    lastSpO2Ms = now;
  }

  // ── Serial CSV output every 500ms ────────────────────
  if (now - lastPrintMs >= PRINT_INTERVAL) {
    lastPrintMs = now;

    long irValue  = particleSensor.getIR();
    long redValue = particleSensor.getRed();
    int  fingerOn = (irValue >= 50000) ? 1 : 0;

    // Always emit DATA line — Python filters by prefix
    Serial.print(F("DATA,"));
    Serial.print(irValue);
    Serial.print(",");
    Serial.print(redValue);
    Serial.print(",");
    Serial.print(beatsPerMinute, 1);
    Serial.print(",");
    Serial.print(beatAvg);
    Serial.print(",");
    Serial.print(spo2Value);
    Serial.print(",");
    Serial.print(spo2Valid);
    Serial.print(",");
    Serial.print(hrValid);
    Serial.print(",");
    Serial.println(fingerOn);
  }

  // ── MQTT publish every 1s ────────────────────────────
  if (now - lastMqttMs >= MQTT_INTERVAL) {
    lastMqttMs = now;

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    char buf[64];
    long irValue = particleSensor.getIR();

    if (irValue >= 50000) {
      int pubHR   = hrValid   ? (int)heartRateValue : beatAvg;
      int pubSpO2 = spo2Valid ? (int)spo2Value      : 0;
      snprintf(buf, sizeof(buf), "%d", pubHR);
      mqttClient.publish(TOPIC_HR, buf);
      snprintf(buf, sizeof(buf), "%d", pubSpO2);
      mqttClient.publish(TOPIC_SPO2, buf);
    }

    snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f",
             accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    mqttClient.publish(TOPIC_ACCEL, buf);

    snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f",
             gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    mqttClient.publish(TOPIC_GYRO, buf);
  }
}

// ─────────────────────────────────────────────────────────
void detectHeartBeat() {
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue)) {
    long delta     = millis() - lastBeat;
    lastBeat       = millis();
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

void fillSpO2Buffer() {
  for (byte i = 0; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2Value, &spo2Valid, &heartRateValue, &hrValid);
}

void recalcSpO2() {
  for (byte i = 25; i < BUFFER_LENGTH; i++) {
    redBuffer[i-25] = redBuffer[i];
    irBuffer[i-25]  = irBuffer[i];
  }
  for (byte i = 75; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available()) particleSensor.check();
    detectHeartBeat();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2Value, &spo2Valid, &heartRateValue, &hrValid);
}
