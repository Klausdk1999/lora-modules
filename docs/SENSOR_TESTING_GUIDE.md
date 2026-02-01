# Sensor Testing Guide - River Level Monitoring

This guide explains how to test the real sensors with your LoRa boards.

## Testing Groups

### 📡 Group A: LilyGo T-Beam AXP2101 v1.2 + TF02-Pro LiDAR

**Best for:** High-accuracy measurements, clear water, indoor testing

| Component | Details |
|-----------|---------|
| **Board** | LilyGo T-Beam AXP2101 v1.2 |
| **Sensor** | TF02-Pro LiDAR |
| **Interface** | UART |
| **Range** | 0.1m - 22m (indoor) |
| **Power** | T-Beam battery + external 5V boost |

**Wiring:**
```
TF02-Pro    →    T-Beam
─────────────────────────
VCC         →    5V boost output
GND         →    GND
TX          →    GPIO 13 (Serial2 RX)
RX          →    GPIO 14 (Serial2 TX)
```

---

### 📡 Group B: Heltec LoRa32 V2 + Ultrasonic

**Best for:** Outdoor waterproof measurements, longer range

| Component | Details |
|-----------|---------|
| **Board** | Heltec WiFi LoRa 32 V2 |
| **Sensor** | HC-SR04 or JSN-SR04T |
| **Interface** | GPIO (Trigger/Echo) |
| **Range** | HC-SR04: 2-400cm, JSN-SR04T: 25-450cm |
| **Power** | External battery board |

**Wiring:**
```
Ultrasonic  →    Heltec
─────────────────────────
VCC         →    5V (or 3.3V for JSN-SR04T)
GND         →    GND
TRIG        →    GPIO 13
ECHO        →    GPIO 12
```

---

## Step-by-Step Setup

### 1. Hardware Assembly

#### For T-Beam + TF02-Pro:
1. Connect the TF02-Pro sensor using 4 wires (VCC, GND, TX, RX)
2. Use GPIO 13 (RX) and GPIO 14 (TX)
3. Connect a 18650 battery to the T-Beam JST connector
4. Use a separate 5V boost module for TF02-Pro power
5. Attach the LoRa antenna

#### For Heltec + Ultrasonic:
1. Connect the ultrasonic sensor using 4 wires (VCC, GND, TRIG, ECHO)
2. Use GPIO 13 for TRIG and GPIO 12 for ECHO
3. Connect your external battery board to provide power
4. Attach the LoRa antenna

### 2. Software Configuration

#### Selecting the Ultrasonic Sensor Type (Heltec only):
In `heltec-lora32-v2/src/main.cpp`, uncomment the appropriate line:

```cpp
// For standard indoor HC-SR04:
#define USE_HCSR04

// For waterproof outdoor JSN-SR04T:
// #define USE_JSNSR04T
```

### 3. Building and Uploading

```bash
# For T-Beam with TF02-Pro:
cd LoRa-River-Monitoring/lilygo-lora32
pio run --target upload

# For Heltec with Ultrasonic:
cd LoRa-River-Monitoring/heltec-lora32-v2
pio run --target upload
```

### 4. Monitoring Serial Output

```bash
# Open serial monitor at 115200 baud
pio device monitor --baud 115200
```

---

## Expected Serial Output

### T-Beam + TF02-Pro:
```
=============================================
LilyGo T-Beam AXP2101 v1.2 - TF02-Pro LiDAR Sensor Node
=============================================

Initializing TF02-Pro LiDAR sensor...
✓ TF02-Pro sensor initialized successfully
  Test reading: 125.0 cm

Setup complete. Joining LoRaWAN network...

--- Sensor Reading ---
Distance: 125.3 cm
Signal Strength: 1542
Temperature: 24 °C
Valid readings: 5/5
----------------------
>>> Sending packet #1: 125.3 cm
```

### Heltec + Ultrasonic:
```
==============================================
Heltec LoRa32 V2 - Ultrasonic Sensor Node
Sensor: HC-SR04
==============================================

Initializing HC-SR04 sensor...
✓ Sensor pins initialized
  Test reading: 87.5 cm

Setup complete. Joining LoRaWAN network...

Reading ultrasonic sensor...
  Reading 1: 87.2 cm
  Reading 2: 87.5 cm
  Reading 3: 87.3 cm
  Reading 4: 87.4 cm
  Reading 5: 87.6 cm

--- Sensor Reading Summary ---
Average Distance: 87.4 cm
Valid readings: 5/5
------------------------------
>>> Sending packet #1: 87.4 cm
```

---

## Payload Format

Both nodes send the same 8-byte payload structure:

| Byte | Field | Description |
|------|-------|-------------|
| 0 | sensorType | 1=TF02-Pro, 2=HC-SR04, 3=JSN-SR04T, 0xFF=Error |
| 1-2 | distanceMm | Distance in millimeters (uint16) |
| 3-4 | signalStrength | Signal quality (LiDAR only) |
| 5 | temperature | Temperature in °C |
| 6 | batteryPercent | Battery level 0-100 |
| 7 | readingCount | Number of valid readings averaged |

### TTN Payload Decoder:
```javascript
function decodeUplink(input) {
  var data = {};
  data.sensorType = input.bytes[0];
  data.distanceMm = (input.bytes[2] << 8) | input.bytes[1];
  data.distanceCm = data.distanceMm / 10.0;
  data.signalStrength = (input.bytes[4] << 8) | input.bytes[3];
  data.temperature = input.bytes[5] > 127 ? input.bytes[5] - 256 : input.bytes[5];
  data.battery = input.bytes[6];
  data.readingCount = input.bytes[7];
  
  var sensorNames = {
    1: "TF02-Pro",
    2: "HC-SR04",
    3: "JSN-SR04T",
    255: "Error"
  };
  data.sensorName = sensorNames[data.sensorType] || "Unknown";
  
  return { data: data };
}
```

---

## Sensor Comparison

| Feature | TF-Luna (LiDAR) | HC-SR04 | JSN-SR04T |
|---------|-----------------|---------|-----------|
| Type | LiDAR (ToF) | Ultrasonic | Ultrasonic |
| Range | 20-800cm | 2-400cm | 25-450cm |
| Accuracy | ±6cm | ±3mm | ±1cm |
| Waterproof | No | No | Yes (IP67) |
| Works on Water | Variable* | Yes | Yes |
| Interface | I2C | GPIO | GPIO |
| Power | 3.3V | 5V | 3.3-5V |
| Cost | ~$15-25 | ~$2-5 | ~$5-10 |

*LiDAR performance on water depends on turbidity and surface conditions.

---

## Troubleshooting

### TF-Luna shows "NOT detected"
- Check I2C wiring (SDA to GPIO 21, SCL to GPIO 22)
- Verify 3.3V power supply
- Try adding 4.7kΩ pull-up resistors on SDA/SCL lines

### Ultrasonic returns 0 or -1
- Verify 5V power (HC-SR04 needs 5V)
- Check TRIG/ECHO connections
- Ensure target is within range and beyond blind zone
- For JSN-SR04T, ensure the waterproof probe cable is connected

### LoRa not joining
- Check antenna is connected
- Verify TTN credentials match your device
- Ensure you're within gateway range
- Check frequency plan (AU915 for Brazil)

---

## Next Steps

1. **Test indoors first** - Verify sensor readings with known distances
2. **Calibrate if needed** - Adjust temperature setting for ultrasonic sensors
3. **Field deployment** - Test in actual river monitoring conditions
4. **Data analysis** - Compare accuracy between sensor types
5. **Long-term testing** - Monitor battery life and reliability



