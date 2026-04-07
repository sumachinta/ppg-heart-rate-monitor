/*
 * ═══════════════════════════════════════════════════════════
 *  ESP32 MAX30102 — RAW SIGNAL ONLY
 *
 *  Outputs only the two raw light values from the sensor:
 *    - IR  : infrared light reflected back from tissue
 *    - Red : red light reflected back from tissue
 *
 *  Serial output format (one line per sample):
 *    DATA,<ir>,<red>
 *
 *  Example:
 *    DATA,187432,31205
 * ═══════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include "MAX30105.h"   // SparkFun MAX3010x library

// ── I2C pins for Adafruit HUZZAH32 ───────────────────────
#define SDA_PIN 21
#define SCL_PIN 22

// ── Sensor object ─────────────────────────────────────────
MAX30105 particleSensor;

// ═══════════════════════════════════════════════════════════
//  SENSOR SETTINGS — things you can tune
// ═══════════════════════════════════════════════════════════

// LED brightness (0–255)
// Higher = more light emitted = stronger reflected signal
// Too low  → IR stays under 50,000 even with finger (bad)
// Too high → signal saturates and clips at 262,144 (bad)
// Good range for finger: 60–200. Start at 120.
const byte LED_BRIGHTNESS = 40;

// How many raw samples the sensor averages before giving you one value
// 1 = no averaging, you get every raw sample (100 per second at sampleRate=100)
// 4 = sensor averages 4 samples → you get 25 per second
// Use 1 if you want to do your own signal processing
const byte SAMPLE_AVERAGE = 1;

// LED mode
// 1 = Red only, 2 = Red + IR (use 2 for SpO2 — we need both channels)
const byte LED_MODE = 2;

// How many raw samples the sensor captures per second internally
// Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
// With SAMPLE_AVERAGE=1 this is also your output rate
// 100 Hz is plenty for HR and SpO2 (heart beats ~1-2 Hz)
const int SAMPLE_RATE = 100;

// Pulse width of the LED flash (microseconds)
// Longer = more light per flash = better signal, but sets ADC bit depth
// 69µs=15bit, 118µs=16bit, 215µs=17bit, 411µs=18bit (max=262144)
// Use 411 for highest resolution
const int PULSE_WIDTH = 411;

// ADC full-scale range
// Options: 2048, 4096, 8192, 16384
// Higher range = less sensitive but handles brighter reflections
// 4096 is a good default
const int ADC_RANGE = 4096;

// ── How often to send a line over Serial (milliseconds) ──
// At SAMPLE_RATE=100 and SAMPLE_AVERAGE=1 the sensor produces
// a new sample every 10ms. Setting this to 10ms sends every sample.
// Setting to 50ms sends every 5th sample (20 lines/sec to Python).
// Lower = more data = smoother plot but more serial traffic.
const uint16_t PRINT_INTERVAL_MS = 10;   // 10ms = full 100Hz to Python

// ── Finger detection threshold ────────────────────────────
// IR value below this = no finger present
// Typical no-finger IR < 5,000. Finger present IR > 50,000.
const long FINGER_THRESHOLD = 50000;

// ─────────────────────────────────────────────────────────
unsigned long lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("# ESP32 MAX30102 Raw Signal Stream"));
  Serial.println(F("# Initializing sensor..."));

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("# ERROR: MAX30102 not found. Check wiring!"));
    while (true) { delay(1000); }
  }

  // Apply all settings defined above
  particleSensor.setup(
    LED_BRIGHTNESS,
    SAMPLE_AVERAGE,
    LED_MODE,
    SAMPLE_RATE,
    PULSE_WIDTH,
    ADC_RANGE
  );

  // Red LED amplitude — controls Red channel brightness separately
  // 0x0A is HEX code for decimal value 10 (good for low-brightness setups)
  // 0x1F is HEX code for decimal value 30, 0xFF is 255 maximum
  // Each step is 0.2mA so 0x0A HEX = 10 units = 2.0mA
  particleSensor.setPulseAmplitudeRed(0x1F);

  // Green LED off — not used for HR or SpO2
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.println(F("# Sensor ready. Place finger on sensor."));
  Serial.println(F("# Output format: DATA,ir,red"));
  Serial.println(F("# finger_on = IR > 50000"));
  Serial.println(F("HEADER,ir,red"));
}

void loop() {
  unsigned long now = millis();

  // Read the latest sample from the sensor FIFO buffer
  // getIR() and getRed() return the most recent ADC count (0–262144)
  long irValue  = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // Send a CSV line at the configured interval
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;

    // Format: DATA,<ir>,<red>
    // ir  = infrared reflected light (primary signal for heart rate)
    // red = red reflected light (needed alongside IR for SpO2)
    Serial.print(F("DATA,"));
    Serial.print(irValue);
    Serial.print(F(","));
    Serial.println(redValue);
  }
}
