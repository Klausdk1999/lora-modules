# Complete System Architecture - River Level Monitoring WSN

## System Overview

This document describes the complete system architecture for the river level monitoring Wireless Sensor Network (WSN) implementation.

## Network Topology

```
                    ┌─────────────────────────────┐
                    │   The Things Network (TTN)    │
                    │   LoRaWAN Network Server      │
                    │                              │
                    │  ┌────────────────────────┐  │
                    │  │  MQTT Broker           │  │
                    │  │  TTN Console           │  │
                    │  │  Data Storage           │  │
                    │  └────────────────────────┘  │
                    └──────────────┬────────────────┘
                                   │
                          Internet Connection
                                   │
                    ┌──────────────▼────────────────┐
                    │   Wisgate Edge Pro Gateway    │
                    │   LoRaWAN Gateway             │
                    │                               │
                    │  ┌─────────────────────────┐  │
                    │  │  LoRa Radio (SX1276/8)   │  │
                    │  │  Frequency: AU915        │  │
                    │  │  Range: Up to 15km       │  │
                    │  └─────────────────────────┘  │
                    └──────────────┬────────────────┘
                                   │
                        LoRaWAN (AU915, 915MHz)
                                   │
            ┌───────────────────────┼───────────────────────┐
            │                       │                       │
    ┌───────▼────────┐      ┌───────▼────────┐      ┌───────▼────────┐
    │   Node 1       │      │   Node 2       │      │  Future Nodes  │
    │  T-Beam AXP2101│      │  Heltec V2     │      │   (Scalable)   │
    └────────────────┘      └────────────────┘      └────────────────┘
```

## Node 1: LilyGo T-Beam AXP2101 v1.2 + TF02-Pro LiDAR

### Hardware Components

```
┌─────────────────────────────────────────────┐
│   LilyGo T-Beam AXP2101 v1.2 Node           │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  ESP32 Dual-Core Microcontroller      │  │
│  │  - Processing: Sensor data, LoRaWAN    │  │
│  │  - Power Management: Deep sleep       │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  Semtech SX1276/8 LoRa Module        │  │
│  │  - Frequency: 915MHz (AU915)          │  │
│  │  - Data Rate: SF7                     │  │
│  │  - TX Power: 14 dBm                   │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  TF02-Pro LiDAR Sensor (UART)        │  │
│  │  - Interface: Serial2 (GPIO 13/14)    │  │
│  │  - Range: 0.1m - 22m (indoor)        │  │
│  │  - Range: 0.1m - 12m (outdoor)       │  │
│  │  - Accuracy: ±1cm (0.1-6m)           │  │
│  │  - Frame Rate: 10Hz (power saving)   │  │
│  │  - Temperature Compensation: Yes      │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  18650 Battery (T-Beam)               │  │
│  │  - Voltage: 3.7V                      │  │
│  │  - Connector: JST-PH 2.0              │  │
│  │  - Estimated Life: 6-12 months        │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  18650 + 5V Boost (TF02-Pro)          │  │
│  │  - 5V output to TF02-Pro VCC          │  │
│  │  - Common ground with T-Beam          │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  LoRa Antenna                         │  │
│  │  - Type: 915MHz                       │  │
│  │  - Connector: U.FL/SMA                │  │
│  └──────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

### Software Architecture

```
┌─────────────────────────────────────────────┐
│         Software Flow (15-minute cycle)     │
└─────────────────────────────────────────────┘

1. Wake from Deep Sleep (RTC Timer)
   │
   ├─> Initialize Serial2 (UART for TF02-Pro)
   │
   ├─> Initialize TF02-Pro Sensor
   │   - Set frame rate to 10Hz
   │   - Clear buffer
   │
   ├─> Read Sensor Data (5 readings)
   │   - Read distance, signal strength, temperature
   │   - Apply temperature compensation
   │   - Average valid readings
   │
   ├─> Read Battery Voltage
   │   - ADC reading (if available)
   │   - Convert to percentage
   │
   ├─> Format Payload (8 bytes)
   │   - Sensor type: 1 (TF02-Pro)
   │   - Distance (mm)
   │   - Signal strength
   │   - Temperature (°C)
   │   - Battery (%)
   │   - Reading count
   │
   ├─> Transmit via LoRaWAN
   │   - Join network (if needed)
   │   - Send payload
   │   - Wait for RX window
   │
   └─> Enter Deep Sleep (15 minutes)
```

## Node 2: Heltec LoRa32 V2 + JSN-SR04T Ultrasonic

### Hardware Components

```
┌─────────────────────────────────────────────┐
│         Heltec LoRa32 V2 Node              │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  ESP32-WROOM-32 Microcontroller      │  │
│  │  - Processing: Sensor data, LoRaWAN  │  │
│  │  - Power Management: Deep sleep     │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  Semtech SX1276/8 LoRa Module        │  │
│  │  - Frequency: 915MHz (AU915)          │  │
│  │  - Data Rate: SF7                     │  │
│  │  - TX Power: 14 dBm                   │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  OLED Display (128x64)               │  │
│  │  - Status information                 │  │
│  │  - Distance readings                  │  │
│  │  - Battery level                      │  │
│  │  - Powered off in sleep                │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  JSN-SR04T Ultrasonic Sensor (GPIO)  │  │
│  │  - Interface: GPIO (TRIG: 13, ECHO: 12)│
│  │  - Range: 25cm - 450cm                │  │
│  │  - Waterproof: IP67 (probe)           │  │
│  │  - Accuracy: ±1cm                      │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  External Battery Board              │  │
│  │  - Input: LiPo 3.7V                  │  │
│  │  - Output: 5V regulated              │  │
│  │  - Battery: 2000mAh                   │  │
│  │  - Estimated Life: 6-12 months        │  │
│  └──────────────────────────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  LoRa Antenna                         │  │
│  │  - Type: 915MHz                       │  │
│  │  - Connector: U.FL/SMA                │  │
│  └──────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

### Software Architecture

```
┌─────────────────────────────────────────────┐
│         Software Flow (15-minute cycle)     │
└─────────────────────────────────────────────┘

1. Wake from Deep Sleep (RTC Timer)
   │
   ├─> Initialize OLED Display
   │   - Power on display
   │   - Show "Waking up..."
   │
   ├─> Initialize Ultrasonic Sensor
   │   - Configure GPIO pins
   │   - Set trigger/echo pins
   │
   ├─> Read Sensor Data (5 readings)
   │   - Trigger sensor
   │   - Measure echo duration
   │   - Calculate distance
   │   - Apply temperature correction
   │   - Average valid readings
   │
   ├─> Read Battery Voltage
   │   - GPIO 37 (voltage divider)
   │   - Convert to percentage
   │
   ├─> Update OLED Display
   │   - Show status
   │   - Show distance
   │   - Show battery level
   │
   ├─> Format Payload (8 bytes)
   │   - Sensor type: 3 (JSN-SR04T)
   │   - Distance (mm)
   │   - Signal strength: 0
   │   - Temperature (°C)
   │   - Battery (%)
   │   - Reading count
   │
   ├─> Transmit via LoRaWAN
   │   - Join network (if needed)
   │   - Send payload
   │   - Wait for RX window
   │
   ├─> Power Off OLED Display
   │
   └─> Enter Deep Sleep (15 minutes)
```

## Data Payload Structure

### 8-Byte Payload Format

```
Byte 0:  Sensor Type
         - 1 = TF02-Pro LiDAR
         - 2 = HC-SR04 Ultrasonic
         - 3 = JSN-SR04T Ultrasonic
         - 0xFF = Error indicator

Byte 1-2: Distance (uint16_t, little-endian)
         - Distance in millimeters
         - Range: 0-65535 mm (0-65.5 meters)

Byte 3-4: Signal Strength (int16_t, little-endian)
         - For LiDAR: Signal flux value
         - For Ultrasonic: 0 (not available)

Byte 5:   Temperature (int8_t, signed)
         - Temperature in Celsius
         - Range: -128 to +127°C
         - Used for compensation (LiDAR)

Byte 6:   Battery Level (uint8_t)
         - Battery percentage (0-100%)
         - For monitoring and maintenance

Byte 7:   Reading Count (uint8_t)
         - Number of valid readings averaged
         - Typically 5 readings
         - Indicates data quality
```

### Example Payload Decoding

**TF02-Pro Reading:**
- Distance: 1250mm (125.0 cm)
- Signal: 1542 (good signal)
- Temperature: 24°C
- Battery: 85%
- Readings: 5

**JSN-SR04T Reading:**
- Distance: 3500mm (350.0 cm)
- Signal: 0 (not available)
- Temperature: 22°C (ambient, for speed of sound)
- Battery: 90%
- Readings: 5

## Power Consumption Analysis

### Active Operation (5-10 seconds per cycle)

| Component | Current | Duration | Energy per Cycle |
|-----------|---------|----------|------------------|
| ESP32 (Active) | 80-240mA | 5-10s | ~1-2.4 mAh |
| LoRa TX | 120mA | 1-2s | ~0.12-0.24 mAh |
| LoRa RX | 15mA | 1-2s | ~0.015-0.03 mAh |
| TF02-Pro | 100mA | 2-5s | ~0.2-0.5 mAh |
| JSN-SR04T | 30mA | 2-5s | ~0.06-0.15 mAh |
| OLED (Heltec) | 20mA | 5-10s | ~0.1-0.2 mAh |
| **Total per Cycle** | | | **~1.5-3.5 mAh** |

### Deep Sleep (15 minutes = 900 seconds)

| Component | Current | Duration | Energy per Cycle |
|-----------|---------|----------|------------------|
| ESP32 (Deep Sleep) | 10µA | 900s | ~0.0025 mAh |
| LoRa (Sleep) | 1µA | 900s | ~0.00025 mAh |
| TF02-Pro (Idle) | 20mA | 900s | ~5 mAh |
| JSN-SR04T (Idle) | 2mA | 900s | ~0.5 mAh |
| **Total per Cycle** | | | **~5.5-6 mAh** |

### Battery Life Calculation

**Total Energy per Cycle:** ~7-9.5 mAh
**Cycles per Day:** 96 (every 15 minutes)
**Daily Consumption:** ~672-912 mAh
**Battery Capacity:** 2000 mAh
**Estimated Battery Life:** 2.2-3 days (continuous operation)

**With Deep Sleep (sensors powered off):**
**Total Energy per Cycle:** ~1.5-3.5 mAh (active) + ~0.003 mAh (sleep) ≈ 1.5-3.5 mAh
**Daily Consumption:** ~144-336 mAh
**Estimated Battery Life:** 6-14 days

**Note:** Actual battery life depends on:
- Sensor power consumption in sleep mode
- Transmission success rate (retries consume more power)
- Environmental temperature (affects battery capacity)
- Battery aging and capacity degradation

**Optimization:** Power sensors off during deep sleep to achieve 6-12 month battery life target.

## Communication Protocol

### LoRaWAN Configuration

- **Frequency Band:** AU915 (915MHz, Americas/Brazil)
- **Protocol:** LoRaWAN Class A
- **Activation:** OTAA (Over-The-Air Activation)
- **Data Rate:** SF7 (Spreading Factor 7)
- **TX Power:** 14 dBm
- **Sub-band:** 1 (Channels 8-15)
- **ADR:** Disabled (fixed data rate)

### Network Server

- **Provider:** The Things Network (TTN)
- **Gateway:** Wisgate Edge Pro
- **Data Access:** MQTT, TTN Console
- **Payload Format:** Custom 8-byte binary

## Deployment Configuration

### Node Positioning

```
                    ┌─────────────────────┐
                    │   Mounting Structure │
                    │   (2-4m above water) │
                    │                      │
                    │  ┌────────────────┐ │
                    │  │  Node 1        │ │
                    │  │  (LilyGo)      │ │
                    │  │  TF02-Pro      │ │
                    │  │  LiDAR         │ │
                    │  └────────────────┘ │
                    │                      │
                    │  ┌────────────────┐ │
                    │  │  Node 2        │ │
                    │  │  (Heltec)      │ │
                    │  │  JSN-SR04T     │ │
                    │  │  Ultrasonic    │ │
                    │  └────────────────┘ │
                    └─────────────────────┘
                              │
                              │ Measurement
                              ▼
                    ┌─────────────────────┐
                    │   River Surface     │
                    └─────────────────────┘
```

### Environmental Considerations

1. **Mounting Height:** 2-4 meters above expected water level
2. **Enclosure:** Weatherproof (IP65/IP67)
3. **Antenna:** Vertical orientation, line-of-sight to gateway
4. **Power:** Battery with optional solar panel
5. **Access:** For maintenance and battery replacement

## System Status

### Completed Components

- ✅ Sensor libraries (TF02-Pro, JSN-SR04T, HC-SR04, TF-Luna)
- ✅ LoRaWAN integration (OTAA, TTN)
- ✅ Power management (deep sleep)
- ✅ Battery monitoring
- ✅ Temperature compensation (TF02-Pro)
- ✅ Data payload formatting
- ✅ Network connectivity validation

### Pending Components

- ⏳ Field deployment
- ⏳ Long-term data collection
- ⏳ Performance analysis
- ⏳ Comparative evaluation

## Next Steps

1. **Field Deployment:**
   - Select deployment site
   - Install sensor nodes
   - Validate connectivity
   - Secure enclosures

2. **Data Collection:**
   - Collect minimum 1 week of data
   - Monitor transmission success
   - Track battery consumption
   - Document environmental conditions

3. **Analysis:**
   - Compare sensor performance
   - Evaluate power efficiency
   - Assess network reliability
   - Generate recommendations
