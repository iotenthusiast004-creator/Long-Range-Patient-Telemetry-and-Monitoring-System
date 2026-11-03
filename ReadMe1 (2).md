# Long-Range Patient Telemetry System (ESP32 + LoRa)

An open-source, long-range wireless patient monitoring system designed for real-time vital signs tracking. Built using **ESP32 microcontrollers** and **Ebyte E32 900MHz LoRa transceivers**, this project wirelessly transmits Heart Rate (BPM), Pulse Oximetry ($\text{SpO}_2$), and Body Temperature from a remote patient node to a nurse station monitoring node.

The receiver unit acts as a standalone wireless gateway, generating its own Wi-Fi Access Point and hosting a responsive web dashboard with live scrolling graphs—requiring no local router or internet connection.

## 🌟 Key Features

* **Long-Range Point-to-Point LoRa Link:** Uses 900 MHz E32-900T20D modules operating in transparent mode for reliable communication over long distances.
* **Real-Time Vitals Acquisition:**
  * **Heart Rate &** $\text{SpO}_2$**:** Powered by the MAX30105 sensor with dual auto-LED brightness adjustments (melanin/skin-tone compensation) and dynamic peak detection.
  * **Body Temperature:** Non-blocking asynchronous sampling via a DS18B20 1-Wire sensor.
* **Dual Display & Monitoring:**
  * **Local Patient Display:** 0.96" SSD1306 OLED screen showing real-time BPM, $\text{SpO}_2$, and a beating heart indicator.
  * **Remote Web Dashboard:** Self-hosted HTTP server serving a lightweight HTML5/Canvas UI with real-time numeric readouts and scrolling historical charts.
* **Bi-Directional Command Protocol:**
  * Query single readings on demand (`SPO2`, `TEMP`).
  * Toggle 2-second interval continuous data streams (`LIVE`, `STOP`).
  * Send remote diagnostic signals (`BLINK`).

## 📐 System Architecture

```
+------------------------------------+           +------------------------------------+
|            PATIENT NODE            |   LoRa    |           NURSE STATION            |
|                                    | (900 MHz) |                                    |
| [MAX30105] --I2C--+                |           | [ESP32] --UART-- [E32-900T20D]     |
| [SSD1306]  --I2C--+-- [ESP32]      | <=======> |   |                                |
| [DS18B20] --GPIO4-+    |           |           |   +-- USB Serial (115200 baud)     |
|                        |           |           |   +-- SoftAP (SpO2-Monitor)        |
| [E32-900T20D] --UART--- +          |           |       Web Dashboard @ 192.168.4.1  |
+------------------------------------+           +------------------------------------+
```

## 🛠 Hardware Required

### **1. Patient / Sender Node**
* ESP32 Development Board
* Ebyte E32-900T20D LoRa Module (900 MHz)
* MAX30105 Pulse Oximeter & Heart Rate Sensor
* DS18B20 Temperature Sensor
* $4.7\text{ k}\Omega$ Resistor (Pull-up for DS18B20)
* 0.96" SSD1306 I2C OLED Display ($128\times 64$)
* Breadboard & Jumper Wires
* Portable Power Bank / 3.7V Li-ion Battery

### **2. Nurse Station / Receiver Node**
* ESP32 Development Board
* Ebyte E32-900T20D LoRa Module (900 MHz)
* Micro-USB / USB-C Cable (for power and Serial Monitor)

## 🔌 Pin Connections & Wiring

### **1. Nurse Station Node Wiring**

| E32 Module Pin | ESP32 Pin | Function / Description |
| :--- | :--- | :--- |
| `VCC` | `3.3V` | Power (3.3V) |
| `GND` | `GND` | Ground |
| `TXD` | `GPIO16` | Hardware Serial RX2 |
| `RXD` | `GPIO17` | Hardware Serial TX2 |
| `M0` | `GND` | Mode 0 (Transparent Transmission) |
| `M1` | `GND` | Mode 0 (Transparent Transmission) |

### **2. Patient Node Wiring**

| Component | Component Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **E32 LoRa** | `VCC` | `3.3V` | Power (3.3V) |
| **E32 LoRa** | `GND` | `GND` | Ground |
| **E32 LoRa** | `TXD` | `GPIO16` | RX2 |
| **E32 LoRa** | `RXD` | `GPIO17` | TX2 |
| **E32 LoRa** | `M0` & `M1` | `GND` | Transparent Mode |
| **MAX30105** | `VCC` | `3.3V` | Sensor Power |
| **MAX30105** | `GND` | `GND` | Ground |
| **MAX30105** | `SDA` | `GPIO22` | Shared I2C SDA Bus |
| **MAX30105** | `SCL` | `GPIO23` | Shared I2C SCL Bus |
| **SSD1306 OLED** | `VCC` | `3.3V` | Display Power |
| **SSD1306 OLED** | `GND` | `GND` | Ground |
| **SSD1306 OLED** | `SDA` | `GPIO22` | Shared I2C SDA Bus |
| **SSD1306 OLED** | `SCL` | `GPIO23` | Shared I2C SCL Bus |
| **DS18B20** | `VDD` (Red) | `3.3V` | Sensor Power |
| **DS18B20** | `GND` (Black) | `GND` | Ground |
| **DS18B20** | `DATA` (Yellow) | `GPIO4` | **Requires $4.7\text{ k}\Omega$ resistor between DATA and 3.3V** |

*Note: MAX30105 (I2C address `0x57`) and SSD1306 (I2C address `0x3C`) share the same I2C lines (`GPIO22`/`GPIO23`) without conflict.*

---

## 🏗️ Step-by-Step Build & Setup Guide

### **Step 1: Hardware Assembly**
1. **Wire the Nurse Station Unit:** Connect the E32 LoRa module to the first ESP32 board using jumper wires as specified in the Nurse Station table. Connect `M0` and `M1` pins directly to `GND`.
2. **Wire the Patient Unit:**
   * Connect the second E32 LoRa module to the second ESP32.
   * Connect both the MAX30105 and SSD1306 OLED to the same I2C lines (`SDA` $\rightarrow$ `GPIO22`, `SCL` $\rightarrow$ `GPIO23`).
   * Connect the DS18B20 temperature sensor to `GPIO4`. Insert a $4.7\text{ k}\Omega$ pull-up resistor between the `DATA` line and `3.3V`.
3. Attach antennas to both E32 LoRa modules before powering them on to prevent RF damage.

---

### **Step 2: Software Environment Setup**
1. Install [Arduino IDE](https://www.arduino.cc/en/software) (v2.0 or higher recommended).
2. Add ESP32 Board support:
   * Go to **File > Preferences**.
   * Add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` to **Additional Boards Manager URLs**.
   * Go to **Tools > Board > Boards Manager**, search for `esp32`, and click **Install**.
3. Install required libraries via **Tools > Manage Libraries**:
   * `SparkFun MAX3010x Pulse and Proximity Sensor Library`
   * `OneWire` (by Paul Stoffregen)
   * `DallasTemperature` (by Miles Burton)
   * `Adafruit GFX Library`
   * `Adafruit SSD1306`

---

### **Step 3: Flashing Firmware**
1. **Flash Nurse Station Gateway:**
   * Open `Spo2_Sender.ino` in Arduino IDE.
   * Select board: **ESP32 Dev Module** (or your specific ESP32 variant).
   * Connect the Nurse Station ESP32 via USB and select its COM Port.
   * Click **Upload**.
2. **Flash Patient Node:**
   * Open `Spo2_Reciever.ino` in Arduino IDE.
   * Connect the Patient Node ESP32 via USB and select its COM Port.
   * Click **Upload**.

---

### **Step 4: Testing & System Verification**
1. **Patient Unit Check:**
   * Power the patient node via a USB power bank.
   * Observe the SSD1306 OLED display; it should show "Place your finger on sensor".
   * Place your index finger on the MAX30105 pulse oximeter. Within 3-5 seconds, heart rate and $\text{SpO}_2$ values will appear alongside a blinking heart icon.
2. **Nurse Station Web Dashboard Access:**
   * Power the nurse station ESP32.
   * On your mobile phone, tablet, or laptop, scan for Wi-Fi networks and connect to:
     * **SSID:** `SpO2-Monitor`
     * **Password:** `12345678`
   * Open any browser and go to `http://192.168.4.1`.
   * Click the **Start** button on the web interface to trigger real-time telemetry streaming over LoRa.

---

## 📡 Serial Command Protocol

| Command | Action | Expected Response |
| :--- | :--- | :--- |
| `BLINK` | Blinks patient node status LED 5 times | `DONE` |
| `SPO2` | Requests instantaneous BPM & $\text{SpO}_2$ | `VITALS:BPM=xx.x,SPO2=yy.y` |
| `TEMP` | Requests instantaneous temperature | `TEMP:xx.x` |
| `LIVE` | Starts 2-second recurring stream | `LIVE:BPM=xx.x,SPO2=yy.y,TEMP=zz.z` |
| `STOP` | Ends active live stream | `STOPPED` |
