/*
  RECEIVER (powered by a power bank)
  ESP32 + E32-900T20D (LoRa module) + MAX30105 pulse oximeter sensor
  + DS18B20 temperature sensor + 0.96" SSD1306 OLED (128x64, I2C)

  LoRa Wiring:
    E32 VCC  -> ESP32 3.3V
    E32 GND  -> ESP32 GND
    E32 TXD  -> ESP32 GPIO16  (RX2)
    E32 RXD  -> ESP32 GPIO17  (TX2)
    E32 M0   -> GND   (Mode 0: transparent transmission)
    E32 M1   -> GND   (Mode 0: transparent transmission)
    E32 AUX  -> not connected (optional)

  MAX30105 Wiring (I2C):
    VIN / VCC -> ESP32 3.3V
    GND       -> ESP32 GND
    SDA       -> ESP32 GPIO22   (I2C_SDA below)
    SCL       -> ESP32 GPIO23   (I2C_SCL below)
    (INT optional, not used)

  OLED Wiring (I2C, shares the SAME bus as the MAX30105):
    VCC -> ESP32 3.3V
    GND -> ESP32 GND
    SDA -> ESP32 GPIO22   (same pin as MAX30105 SDA)
    SCL -> ESP32 GPIO23   (same pin as MAX30105 SCL)
    (MAX30105 = address 0x57, OLED = 0x3C, so they coexist on one bus)

  DS18B20 Wiring (1-Wire):
    VDD  (red)    -> ESP32 3.3V
    GND  (black)  -> ESP32 GND
    DATA (yellow) -> ESP32 GPIO4
    4.7k pull-up resistor between DATA and 3.3V (required)

  Requires libraries (install via Arduino Library Manager):
    "SparkFun MAX3010x Pulse and Proximity Sensor Library"
    (or SparkFun MAX30105)
    "OneWire"            (Paul Stoffregen)
    "DallasTemperature"  (Miles Burton)
    "Adafruit GFX Library"
    "Adafruit SSD1306"

  Behavior:
    - Continuously samples MAX30105 and maintains live BPM + SpO2 estimates
      (same dual auto-LED + peak detection algorithm as the AikyaNova sketch).
    - Listens for "BLINK" over LoRa -> blinks the blue LED 5x -> replies "DONE"
    - Listens for "SPO2"  over LoRa -> replies current values:
          VITALS:BPM=xx.x,SPO2=yy.y
        or VITALS:NOFINGER  /  VITALS:ERROR
    - Listens for "TEMP"  over LoRa -> replies current temperature:
          TEMP:xx.x      (degrees Celsius)
        or TEMP:ERROR    (sensor missing / disconnected)
    - Listens for "LIVE"  over LoRa -> starts streaming BPM + SpO2 + temperature
        together every 2 s:   LIVE:BPM=xx.x,SPO2=yy.y,TEMP=zz.z
        (a value the sensor can't provide is sent as NA)
    - Listens for "STOP"  over LoRa -> stops the stream, replies "STOPPED"
      (BLINK / SPO2 / TEMP only reply when requested; only LIVE streams)
    - The OLED always shows the live BPM and SpO2 (with a heart that blinks
      on every detected beat), independent of any LoRa command.
    - The DS18B20 is read asynchronously (no waiting for the ~750 ms
      conversion), so BPM / SpO2 sampling is never stalled.
*/

#include <Wire.h>
#include "MAX30105.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ========================================================================
// LoRa & LED
// ========================================================================
#define E32_RX_PIN 16    // ESP32 RX2 <- E32 TXD
#define E32_TX_PIN 17    // ESP32 TX2 -> E32 RXD
#define E32_BAUD   9600  // E32 default baud rate (transparent mode)

#define LED_PIN 2        // Onboard blue LED (GPIO2 on most DOIT DevKit V1 boards;
                         // some clones use another pin, the boot self-test shows it)

HardwareSerial LoRaSerial(2);
MAX30105 sensor;
bool sensorOk = false;

// ========================================================================
// I2C BUS (shared by MAX30105 and the OLED)
// ========================================================================
#define I2C_SDA 22
#define I2C_SCL 23

// ========================================================================
// OLED DISPLAY
// ========================================================================
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1     // OLED RST not wired to a GPIO
#define SCREEN_ADDRESS 0x3C   // change to 0x3D if the display isn't found

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool     displayOk        = false;
bool     fingerPresent    = false;   // updated by processSensor()
uint32_t heartBeatTimer   = 0;       // heart icon stays visible until this time
uint32_t lastDisplayMs    = 0;
const uint32_t DISPLAY_INTERVAL_MS = 100;  // ~10 fps; keeps I2C time low so
                                           // MAX30105 sampling isn't disturbed

// ========================================================================
// DS18B20 TEMPERATURE SENSOR
// ========================================================================
#define ONE_WIRE_PIN 4   // DS18B20 DATA pin (with 4.7k pull-up to 3.3V)

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature tempSensor(&oneWire);

bool  tempOk            = false;   // sensor found at startup
bool  tempValid         = false;   // we have at least one good reading
float tempC             = 0;       // latest temperature in Celsius
bool  tempConverting    = false;
uint32_t tempRequestAt  = 0;
const uint32_t TEMP_CONVERSION_MS = 800;   // 12-bit conversion needs ~750 ms
const uint32_t TEMP_INTERVAL_MS   = 1000;  // refresh rate

// ========================================================================
// LIVE STREAM MODE
// ========================================================================
bool liveMode = false;
uint32_t lastLiveSentAt = 0;
const uint32_t LIVE_INTERVAL_MS = 2000;  // LoRa airtime is slow; 2 s is safe

// ========================================================================
// DUAL AUTO-LED BRIGHTNESS (MELANIN COMPENSATION)
// ========================================================================
byte irPower  = 40;
byte redPower = 40;
unsigned long lastAdjustTime = 0;

// ========================================================================
// DC REMOVAL & BASELINE TRACKING
// ========================================================================
int32_t dcIR  = 0;
int32_t dcRED = 0;
const int SHIFT_DC = 4;       // EMA filter: >> 4 ≡ divide by 16

// ========================================================================
// PEAK DETECTION & BPM VARIABLES
// ========================================================================
int32_t prev = 0, prev2 = 0;
uint32_t lastBeatMs        = 0;
uint8_t  beatCount         = 0;
float bpm = 0;

float env = 0;
const float envAlpha = 0.95f;

const uint32_t REFRACTORY_MS = 400;
uint32_t lastBeatDetectedMs   = 0;
const uint32_t BPM_TIMEOUT_MS = 3000;

const float MIN_SIGNAL_QUALITY = 8.0f;

// ========================================================================
// WARMUP
// ========================================================================
bool     warmupDone  = false;
uint32_t warmupStart = 0;
const uint32_t WARMUP_MS = 500;

// ========================================================================
// SpO2 ESTIMATION
// ========================================================================
float spo2   = 0;
float Rratio = 0;

int32_t irMax  = INT32_MIN, irMin  = INT32_MAX;
int32_t redMax = INT32_MIN, redMin = INT32_MAX;

uint32_t lastSpo2DetectedMs    = 0;
const uint32_t SPO2_TIMEOUT_MS = 10000;

// ========================================================================
// HELPERS
// ========================================================================
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void resetState() {
  bpm   = 0;
  spo2  = 0;
  dcIR  = sensor.getIR();
  dcRED = sensor.getRed();
  env   = 0;
  irMax = INT32_MIN; irMin = INT32_MAX;
  redMax = INT32_MIN; redMin = INT32_MAX;
  lastBeatMs         = millis();
  lastBeatDetectedMs = millis();
  lastSpo2DetectedMs = millis();
  beatCount  = 0;
  warmupDone = false;
  warmupStart = 0;
  irPower = 40; redPower = 40;
  sensor.setPulseAmplitudeIR(irPower);
  sensor.setPulseAmplitudeRed(redPower);
}

// Draws a filled 13x11 px heart at (x, y) = top-left of its bounding box.
void drawHeart(int16_t x, int16_t y, uint16_t color) {
  display.fillCircle(x + 3,  y + 3, 3, color);  // left lobe
  display.fillCircle(x + 9,  y + 3, 3, color);  // right lobe
  display.fillTriangle(x, y + 4, x + 12, y + 4, x + 6, y + 10, color);
}

// ------------------------------------------------------------------------
// Non-blocking LED blink (so the OLED, sensor and LIVE stream keep running)
// ------------------------------------------------------------------------
bool     blinking          = false;
int      blinkTogglesLeft  = 0;
uint32_t blinkIntervalMs   = 0;
uint32_t blinkLastToggleMs = 0;
bool     blinkLedState     = false;

void startBlink(int times, uint32_t intervalMs) {
  blinking          = true;
  blinkTogglesLeft  = times * 2;      // ON + OFF per blink
  blinkIntervalMs   = intervalMs;
  blinkLedState     = false;
  blinkLastToggleMs = millis() - intervalMs;   // first toggle happens immediately
}

void updateBlink() {
  if (!blinking) return;
  if (millis() - blinkLastToggleMs < blinkIntervalMs) return;

  if (blinkTogglesLeft > 0) {
    blinkLedState = !blinkLedState;
    digitalWrite(LED_PIN, blinkLedState ? HIGH : LOW);
    blinkTogglesLeft--;
    blinkLastToggleMs = millis();
  } else {
    // Finished: LED off, report back to the sender
    digitalWrite(LED_PIN, LOW);
    blinking = false;
    delay(100);
    LoRaSerial.println("DONE");
    LoRaSerial.flush();
    Serial.println("[BLINK] Finished");
    Serial.println("[TX] Sent feedback: DONE");
  }
}

// ========================================================================
// SETUP
// ========================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // boot self-test: LED should light for 0.5 s
  delay(500);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);  // optional debug
  Serial.print(F("LED self-test done on GPIO"));
  Serial.println(LED_PIN);

  // I2C bus shared by MAX30105 and the OLED
  Wire.begin(I2C_SDA, I2C_SCL);

  // OLED (periphBegin=false so the library keeps our custom I2C pins)
  displayOk = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, true, false);
  if (displayOk) {
    Serial.println(F("OLED ready."));
  } else {
    Serial.println(F("SSD1306 not found. Check wiring / address (0x3C or 0x3D)."));
  }

  LoRaSerial.begin(E32_BAUD, SERIAL_8N1, E32_RX_PIN, E32_TX_PIN);

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30105 not found. Check wiring!"));
    sensorOk = false;
    // Still continue so BLINK still works; SPO2 will return ERROR
  } else {
    // sampleRate=400, avg=8 → 50 Hz effective; pulseWidth=411 µs → 18-bit
    sensor.setup(0, 8, 2, 400, 411, 4096);
    sensor.setPulseAmplitudeIR(irPower);
    sensor.setPulseAmplitudeRed(redPower);
    sensorOk = true;
    Serial.println(F("MAX30105 ready."));
  }

  // DS18B20
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);  // async: don't block the loop
  tempOk = (tempSensor.getDeviceCount() > 0);
  if (tempOk) {
    Serial.println(F("DS18B20 ready."));
  } else {
    Serial.println(F("DS18B20 not found. Check wiring and 4.7k pull-up!"));
  }

  Serial.println("=== LoRa Receiver + MAX30105 (BPM/SpO2) + DS18B20 Ready ===");
  Serial.println("Commands: BLINK | SPO2 | TEMP | LIVE | STOP");
}

// ========================================================================
// CONTINUOUS SENSOR PROCESSING (called every loop)
// ========================================================================
void processSensor() {
  if (!sensorOk) {
    // Sensor missing – leave bpm/spo2 at 0
    return;
  }

  int32_t irRaw  = sensor.getIR();
  int32_t redRaw = sensor.getRed();

  fingerPresent = (irRaw >= 20000);   // used by the OLED

  // Finger-absent check
  if (irRaw < 20000 && warmupDone) {
    resetState();
    return;
  }

  // Warmup window after placement
  if (!warmupDone) {
    if (warmupStart == 0) warmupStart = millis();
    if (millis() - warmupStart >= WARMUP_MS) {
      warmupDone  = true;
      warmupStart = 0;
    }
  }

  // Dual auto-LED brightness
  if (millis() - lastAdjustTime > 50) {
    bool adjusted = false;

    if      (irRaw < 60000  && irPower < 250) { irPower  += 2; adjusted = true; }
    else if (irRaw > 220000 && irPower > 5)   { irPower   = (irPower > 10) ? irPower - 5 : 5; adjusted = true; }
    else if (irRaw > 180000 && irPower > 5)   { irPower  -= 2; adjusted = true; }

    if      (redRaw < 60000  && redPower < 250) { redPower  += 2; adjusted = true; }
    else if (redRaw > 220000 && redPower > 5)   { redPower   = (redPower > 10) ? redPower - 5 : 5; adjusted = true; }
    else if (redRaw > 180000 && redPower > 5)   { redPower  -= 2; adjusted = true; }

    if (adjusted) {
      sensor.setPulseAmplitudeIR(irPower);
      sensor.setPulseAmplitudeRed(redPower);
    }
    lastAdjustTime = millis();
  }

  // DC removal & AC extraction
  dcIR  = dcIR  + ((irRaw  - dcIR)  >> SHIFT_DC);
  dcRED = dcRED + ((redRaw - dcRED) >> SHIFT_DC);

  int32_t irAC  = dcIR  - irRaw;
  int32_t redAC = dcRED - redRaw;

  if (irAC > irMax)   irMax  = irAC;
  if (irAC < irMin)   irMin  = irAC;
  if (redAC > redMax) redMax = redAC;
  if (redAC < redMin) redMin = redAC;

  int32_t x = irAC / 8;

  // Dynamic threshold
  float absx = (x >= 0) ? x : -x;
  env = envAlpha * env + (1.0f - envAlpha) * absx;
  float thresh = 0.5f * env;

  // Peak detection
  bool isPeak = (prev > prev2) && (prev > x) && (prev > thresh) && (env >= MIN_SIGNAL_QUALITY);

  if (isPeak) {
    uint32_t now = millis();
    uint32_t dt  = now - lastBeatMs;

    if (dt > REFRACTORY_MS) {
      beatCount++;
      lastBeatMs         = now;
      lastBeatDetectedMs = now;
      heartBeatTimer     = now + 150;   // blink the OLED heart for 150 ms

      if (beatCount >= 2) {
        float instBpm = 60000.0f / dt;
        if (bpm == 0) {
          bpm = instBpm;
        } else if (instBpm > bpm * 1.4f) {
          bpm = 0.98f * bpm + 0.02f * instBpm;
        } else {
          bpm = 0.70f * bpm + 0.30f * instBpm;
        }

        // SpO2 ratio-of-ratios
        float acIRAmp  = (float)(irMax  - irMin);
        float acREDAmp = (float)(redMax - redMin);
        float dcIrF    = (float)dcIR;
        float dcRedF   = (float)dcRED;

        if (acIRAmp > 1 && acREDAmp > 1 && dcIrF > 1 && dcRedF > 1) {
          float nir  = acIRAmp / dcIrF;
          float nred = acREDAmp / dcRedF;
          if (nir > 0.000001f) {
            Rratio = nred / nir;
            float spo2Inst = 110.0f - 25.0f * Rratio;
            spo2Inst = clampf(spo2Inst, 70.0f, 100.0f);
            if (spo2 == 0) spo2 = spo2Inst;
            spo2 = 0.80f * spo2 + 0.20f * spo2Inst;
            lastSpo2DetectedMs = now;
          }
        }
      }

      irMax = INT32_MIN; irMin = INT32_MAX;
      redMax = INT32_MIN; redMin = INT32_MAX;
    }
  }

  // Timeouts
  if (bpm  > 0 && (millis() - lastBeatDetectedMs  > BPM_TIMEOUT_MS))  bpm  = 0;
  if (spo2 > 0 && (millis() - lastSpo2DetectedMs  > SPO2_TIMEOUT_MS)) spo2 = 0;

  prev2 = prev;
  prev  = x;
}

// ========================================================================
// DS18B20 NON-BLOCKING UPDATE (called every loop)
// ========================================================================
void processTemperature() {
  if (!tempOk) return;

  uint32_t now = millis();

  if (!tempConverting) {
    if (now - tempRequestAt >= TEMP_INTERVAL_MS) {
      tempSensor.requestTemperatures();   // returns immediately
      tempRequestAt  = now;
      tempConverting = true;
    }
  } else if (now - tempRequestAt >= TEMP_CONVERSION_MS) {
    float t = tempSensor.getTempCByIndex(0);
    tempConverting = false;
    if (t == DEVICE_DISCONNECTED_C || t < -55.0f || t > 125.0f) {
      tempValid = false;   // sensor unplugged or bad read
    } else {
      tempC     = t;
      tempValid = true;
    }
  }
}

// ========================================================================
// OLED UPDATE (called every loop, throttled to DISPLAY_INTERVAL_MS)
// ========================================================================
void updateDisplay() {
  if (!displayOk) return;

  uint32_t now = millis();
  if (now - lastDisplayMs < DISPLAY_INTERVAL_MS) return;
  lastDisplayMs = now;

  display.clearDisplay();
  display.setTextWrap(false);
  display.setTextColor(SSD1306_WHITE);

  // MAX30105 missing
  if (!sensorOk) {
    display.setTextSize(2);
    display.setCursor(0, 10);
    display.print(F("Sensor"));
    display.setCursor(0, 30);
    display.print(F("Error!"));
    display.display();
    return;
  }

  // No finger on the sensor
  if (!fingerPresent) {
    display.setTextSize(2);
    display.setCursor(0, 4);
    display.print(F("Place your"));
    display.setCursor(0, 24);
    display.print(F("finger on"));
    display.setCursor(0, 44);
    display.print(F("sensor"));
    display.display();
    return;
  }

  // --- Headers ---
  display.setTextSize(2);
  display.setCursor(0, 5);  display.print(F("BPM"));
  display.setCursor(72, 5); display.print(F("SpO2"));

  // --- BPM value ---
  display.setTextSize(3);
  display.setCursor(0, 30);
  if (bpm > 20 && bpm < 250) {
    display.print((int)bpm);
  } else if (env < MIN_SIGNAL_QUALITY) {
    display.setTextSize(1);
    display.setCursor(0, 30);
    display.print(F("LOW"));
    display.setCursor(0, 42);
    display.print(F("SIGNAL"));
  } else {
    display.print(F("--"));
  }

  // --- SpO2 value ---
  display.setTextSize(3);
  display.setCursor(72, 30);
  if (spo2 > 70) {
    display.print((int)spo2);
    display.setTextSize(1);
    display.setCursor(117, 30);
    display.print(F("%"));
  } else {
    display.print(F("--"));
  }

  // --- Heart symbol (bottom centre, blinks on each detected beat) ---
  if (now < heartBeatTimer) {
    drawHeart(47, 51, SSD1306_WHITE);
  }

  display.display();
}

// ========================================================================
// COMMAND HANDLING
// ========================================================================
void handleIncomingCommands() {
  if (LoRaSerial.available()) {
    String cmd = LoRaSerial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.length() == 0) return;

    Serial.print("[RX] Received: ");
    Serial.println(cmd);

    if (cmd == "BLINK") {
      if (blinking) {
        Serial.println("[BLINK] Already blinking, ignored");
      } else {
        Serial.print("[BLINK] Started on GPIO");
        Serial.println(LED_PIN);
        startBlink(5, 300);   // 5 blinks, 300 ms on / 300 ms off; DONE sent when finished
      }
    }
    else if (cmd == "SPO2") {
      sendVitalsNow();
    }
    else if (cmd == "TEMP") {
      sendTemperatureNow();
    }
    else if (cmd == "LIVE") {
      liveMode = true;
      lastLiveSentAt = 0;   // send the first sample right away
      Serial.println("[LIVE] Streaming started");
    }
    else if (cmd == "STOP") {
      liveMode = false;
      delay(50);
      LoRaSerial.println("STOPPED");
      LoRaSerial.flush();
      Serial.println("[LIVE] Streaming stopped");
    }
  }
}

// Sends one combined BPM + SpO2 + TEMP sample while live mode is on
void processLiveStream() {
  if (!liveMode) return;
  uint32_t now = millis();
  if (lastLiveSentAt != 0 && now - lastLiveSentAt < LIVE_INTERVAL_MS) return;
  lastLiveSentAt = now;

  bool fingerOk = sensorOk && (sensor.getIR() >= 20000);

  String msg = "LIVE:BPM=";
  msg += (fingerOk && bpm  > 0) ? String(bpm, 1)  : String("NA");
  msg += ",SPO2=";
  msg += (fingerOk && spo2 > 0) ? String(spo2, 1) : String("NA");
  msg += ",TEMP=";
  msg += (tempOk && tempValid)  ? String(tempC, 1) : String("NA");

  LoRaSerial.println(msg);
  Serial.print("[TX] Sent: ");
  Serial.println(msg);
}

void sendTemperatureNow() {
  String msg;
  if (!tempOk || !tempValid) {
    msg = "TEMP:ERROR";
  } else {
    msg = "TEMP:" + String(tempC, 1);
  }

  delay(100);  // let the module settle
  LoRaSerial.println(msg);
  LoRaSerial.flush();

  Serial.print("[TX] Sent: ");
  Serial.println(msg);
}

void sendVitalsNow() {
  if (!sensorOk) {
    delay(50);
    LoRaSerial.println("VITALS:ERROR");
    LoRaSerial.flush();
    Serial.println("[TX] Sent: VITALS:ERROR");
    return;
  }

  int32_t ir = sensor.getIR();
  if (ir < 20000) {
    delay(50);
    LoRaSerial.println("VITALS:NOFINGER");
    LoRaSerial.flush();
    Serial.println("[TX] Sent: VITALS:ERROR");
    return;
  }

  // Use the continuously updated values (kept live by processSensor)
  String msg = "VITALS:BPM=" + String(bpm, 1) + ",SPO2=" + String(spo2, 1);

  delay(100);  // let the module settle
  LoRaSerial.println(msg);
  LoRaSerial.flush();

  Serial.print("[TX] Sent: ");
  Serial.println(msg);
}

// ========================================================================
// MAIN LOOP
// ========================================================================
void loop() {
  processSensor();          // keep BPM / SpO2 estimates live
  processTemperature();     // keep DS18B20 reading fresh (non-blocking)
  updateDisplay();          // show BPM / SpO2 on the OLED
  updateBlink();            // run the LED blink without blocking
  handleIncomingCommands(); // answer BLINK / SPO2 / TEMP / LIVE / STOP
  processLiveStream();      // stream combined readings while LIVE is on
}