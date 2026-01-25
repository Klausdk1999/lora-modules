# Hardware Setup Checklist
## Files to Review Before Starting Hardware Assembly

This document lists all the files you need to check before starting hardware assembly and connections.

---

## 📋 Essential Files for Hardware Setup

### 1. **Wiring Diagrams & Pin Configuration**
**File:** `docs/wiring.md`
- **Purpose:** Complete wiring diagrams for all sensors
- **Contains:**
  - TF02-Pro LiDAR sensor pinout and connections
  - JSN-SR04T ultrasonic sensor pinout
  - DHT11 temperature sensor connections
  - Physical connection diagrams
  - Power configuration diagrams
  - Pin summary tables for both boards

### 2. **Heltec Node Documentation**
**File:** `heltec-lora32-v2/README.md`
- **Purpose:** Complete hardware specification for Heltec board
- **Key Information:**
  - Pin configuration (GPIO 13/12 for ultrasonic, GPIO 27 for DHT11)
  - Battery monitoring pin (GPIO 37)
  - LoRa module pins (integrated)
  - Wiring instructions
  - Academic references for sensor validation

### 3. **LilyGo Node Documentation**
**File:** `lilygo-lora32/README.md`
- **Purpose:** Complete hardware specification for LilyGo board
- **Key Information:**
  - TF02-Pro UART pin configuration (GPIO 13/14)
  - LoRa module pin variations by model
  - Battery monitoring pin (GPIO 35)
  - Important: Board model identification guide

### 4. **Heltec Source Code - Pin Definitions**
**File:** `heltec-lora32-v2/src/main.cpp`
- **Lines to Check:**
  - Lines 92-101: Pin definitions
    ```cpp
    #define ULTRASONIC_TRIG_PIN     13
    #define ULTRASONIC_ECHO_PIN     12
    #define DHT_PIN                 27      // DHT11 DATA pin
    #define BATTERY_ADC_PIN         37
    ```
  - Lines 85-90: LoRa module pins (lmic_pins structure)

### 5. **LilyGo Source Code - Pin Definitions**
**File:** `lilygo-lora32/src/main.cpp`
- **Lines to Check:**
  - Lines 77-82: LoRa module pins (lmic_pins structure)
  - Lines 85-86: TF02-Pro UART pins
    ```cpp
    #define TF02_RX_PIN     13  // GPIO 13 for Serial2 RX
    #define TF02_TX_PIN     14  // GPIO 14 for Serial2 TX
    ```
  - Line 89: Battery monitoring pin

### 6. **Quick Start Guide**
**File:** `QUICK_START.md`
- **Purpose:** Step-by-step setup guide
- **Contains:** Verification checklist, troubleshooting tips

### 7. **Sensor Library Documentation**
**File:** `lib/sensors/README.md`
- **Purpose:** Sensor specifications and usage
- **Contains:** Temperature compensation requirements, sensor ranges

---

## 🔌 Quick Pin Reference

### Heltec WiFi LoRa 32 V2 (Ultrasonic Node)

| Component | Pin | GPIO | Notes |
|-----------|-----|------|-------|
| **JSN-SR04T Ultrasonic** |
| TRIG | - | GPIO 13 | Trigger pin |
| ECHO | - | GPIO 12 | Echo pin |
| VCC | - | 5V | Power (3.3V also works) |
| GND | - | GND | Ground |
| **DHT11 Temperature** |
| DATA | - | GPIO 27 | Data pin (or GPIO 25) |
| VCC | - | 3.3V or 5V | Power |
| GND | - | GND | Ground |
| **LoRa Module (Integrated)** |
| NSS | - | GPIO 18 | Chip select |
| RST | - | GPIO 14 | Reset |
| DIO0 | - | GPIO 26 | Digital I/O 0 |
| DIO1 | - | GPIO 35 | Digital I/O 1 |
| DIO2 | - | GPIO 34 | Digital I/O 2 |
| **Battery Monitoring** |
| Battery ADC | - | GPIO 37 | VBAT_SENSE |

### LilyGo T-Beam AXP2101 v1.2 (LiDAR Node)

| Component | Pin | GPIO | Notes |
|-----------|-----|------|-------|
| **TF02-Pro LiDAR (UART)** |
| TX | - | GPIO 13 | Serial2 RX on ESP32 |
| RX | - | GPIO 14 | Serial2 TX on ESP32 |
| VCC | - | 5V | Power from external boost |
| GND | - | GND | Ground |
| **LoRa Module (Integrated)** |
| NSS | - | GPIO 18 | Chip select |
| RST | - | GPIO 23 | Reset (varies by model) |
| DIO0 | - | GPIO 26 | Digital I/O 0 |
| DIO1 | - | GPIO 33 | Digital I/O 1 |
| DIO2 | - | GPIO 32 | Digital I/O 2 |
| **Battery Monitoring** |
| Battery ADC | - | GPIO 35 | Varies by model |

**⚠️ IMPORTANT:** LilyGo pin configurations vary by model. Always verify your specific board model before wiring!

---

## ✅ Pre-Wiring Checklist

Before starting hardware assembly, verify:

### Documentation Review
- [ ] Read `docs/wiring.md` completely
- [ ] Reviewed `heltec-lora32-v2/README.md`
- [ ] Reviewed `lilygo-lora32/README.md`
- [ ] Identified your specific LilyGo board model
- [ ] Confirmed pin configurations match your hardware

### Code Configuration
- [ ] Verified pin definitions in `heltec-lora32-v2/src/main.cpp` (lines 92-101)
- [ ] Verified pin definitions in `lilygo-lora32/src/main.cpp` (lines 77-89)
- [ ] Checked LoRa module pins match your board model (LilyGo only)
- [ ] Confirmed DHT11 pin selection (GPIO 27 or alternative)

### Hardware Preparation
- [ ] All sensors available (TF02-Pro, JSN-SR04T, DHT11)
- [ ] Boards available (Heltec V2, LilyGo LoRa32)
- [ ] Appropriate power supplies/batteries
- [ ] Jumper wires and connectors
- [ ] Multimeter for testing connections

### Tools Needed
- [ ] Soldering iron (if needed for permanent connections)
- [ ] Breadboard or prototype board
- [ ] USB cables for programming
- [ ] Serial monitor access (PlatformIO or Arduino IDE)

---

## 🔍 Key Wiring Points to Double-Check

### 1. **DHT11 Temperature Sensor (NEW - Recently Added)**
- **Location:** Must be physically near ultrasonic transducer
- **Pin:** GPIO 27 (Heltec) - check for conflicts
- **Power:** 3.3V or 5V (DHT11 supports both)
- **Critical:** Without this, ultrasonic readings will have reduced accuracy

### 2. **TF02-Pro UART Connection**
- **TX (sensor) → RX (ESP32):** GPIO 13 (Serial2 RX)
- **RX (sensor) → TX (ESP32):** GPIO 14 (Serial2 TX)
- **Note:** UART connection, not I2C - verify Serial2 initialization

### 3. **JSN-SR04T Ultrasonic**
- **Works at 3.3V:** No voltage divider needed (unlike HC-SR04)
- **Blind zone:** <25cm - ensure mounting height accounts for this

### 4. **Battery Monitoring**
- **Heltec:** GPIO 37 has built-in voltage divider
- **LilyGo:** GPIO 35 (varies by model - check your board!)

### 5. **TF02-Pro Power (Two-Battery Setup)**
- **Battery 1:** 18650 to T-Beam battery connector
- **Battery 2:** 18650 to 5V boost converter → TF02-Pro VCC
- **Common ground:** Boost GND ↔ T-Beam GND ↔ Sensor GND

### 5. **LoRa Antenna**
- **Required:** Must be connected before powering on
- **Frequency:** Match your region (AU915 for Brazil)
- **Type:** 915MHz or 868MHz depending on region

---

## 📝 Wiring Sequence Recommendation

1. **Start with Power Only**
   - Connect board to USB for testing
   - Verify board powers on (LED indicators)

2. **Add Temperature Sensor (Heltec only)**
   - Connect DHT11 first (simplest sensor)
   - Test in code to verify detection
   - Serial monitor should show temperature readings

3. **Add Distance Sensor**
   - Connect TF02-Pro (LilyGo) or JSN-SR04T (Heltec)
   - Test sensor readings
   - Verify temperature compensation working (Heltec)

4. **Add Battery (if using battery power)**
   - Connect battery after all sensors working
   - Test battery monitoring function

5. **Final Integration**
   - Connect LoRa antenna
   - Test LoRaWAN join
   - Verify complete transmission cycle

---

## 🚨 Common Wiring Mistakes to Avoid

1. **Reversed UART pins:** TX→RX and RX→TX (correct for UART)
2. **Wrong GPIO numbers:** Double-check pin definitions in code
3. **Missing DHT11:** Temperature compensation won't work (Heltec node)
4. **LoRa antenna not connected:** Can damage LoRa module
5. **Power supply issues:** Ensure adequate current (sensors need 30-100mA each)
6. **Ground loops:** Ensure all GND connections are common

---

## 📚 Additional Resources

- **PlatformIO Configuration:** Check `platformio.ini` files for library dependencies
- **TTN Setup:** See `heltec-lora32-v2/TTN_DEVICE_SETUP_GUIDE.md` for network configuration
- **Power Analysis:** See `docs/wiring.md` for power consumption estimates
- **Troubleshooting:** See `QUICK_START.md` troubleshooting section

---

## 🎯 Next Steps After Hardware Setup

1. Test each sensor individually
2. Verify LoRaWAN join to network server
3. Calibrate sensors at known distances
4. Test temperature compensation (Heltec)
5. Verify battery monitoring
6. Test deep sleep functionality
7. Run extended test (24+ hours)

---

**Last Updated:** After DHT11 temperature sensor integration
**Commit:** b603551
