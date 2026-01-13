# System Deployment Schema - River Level Monitoring WSN

This document provides a visual and textual description of the complete system architecture and deployment configuration.

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    River Level Monitoring WSN                    │
└─────────────────────────────────────────────────────────────────┘

                    ┌─────────────────────┐
                    │  Wisgate Edge Pro   │
                    │   LoRaWAN Gateway   │
                    │                     │
                    │  ┌───────────────┐  │
                    │  │  LoRa Radio   │  │
                    │  │  (SX1276/8)   │  │
                    │  └───────────────┘  │
                    │                     │
                    │  ┌───────────────┐  │
                    │  │  Network      │  │
                    │  │  Interface    │  │
                    │  └───────────────┘  │
                    └──────────┬───────────┘
                              │
                    ┌─────────┴─────────┐
                    │                   │
                    │   LoRaWAN         │
                    │   (AU915)         │
                    │                   │
        ┌───────────┴──────────┐        └───────────────┐
        │                      │                        │
┌───────▼────────┐    ┌────────▼─────────┐    ┌─────────▼────────┐
│  Node 1        │    │  Node 2          │    │  Future Nodes   │
│  LilyGo LoRa32 │    │  Heltec LoRa32   │    │  (Scalable)     │
│                │    │  V2              │    │                 │
│  ┌──────────┐  │    │  ┌────────────┐  │    │                 │
│  │ ESP32    │  │    │  │ ESP32      │  │    │                 │
│  │ MCU      │  │    │  │ MCU        │  │    │                 │
│  └────┬─────┘  │    │  └─────┬──────┘  │    │                 │
│       │        │    │        │         │    │                 │
│  ┌────▼─────┐  │    │  ┌─────▼──────┐  │    │                 │
│  │ LoRa     │  │    │  │ LoRa       │  │    │                 │
│  │ SX1276/8 │  │    │  │ SX1276/8   │  │    │                 │
│  └──────────┘  │    │  └────────────┘  │    │                 │
│                │    │                   │    │                 │
│  ┌──────────┐  │    │  ┌────────────┐  │    │                 │
│  │ TF02-Pro │  │    │  │ JSN-SR04T  │  │    │                 │
│  │ LiDAR    │  │    │  │ Ultrasonic │  │    │                 │
│  │ (UART)   │  │    │  │ (GPIO)     │  │    │                 │
│  └──────────┘  │    │  └────────────┘  │    │                 │
│                │    │                   │    │                 │
│  ┌──────────┐  │    │  ┌────────────┐  │    │                 │
│  │ Battery  │  │    │  │ Battery    │  │    │                 │
│  │ (Direct) │  │    │  │ (External) │  │    │                 │
│  └──────────┘  │    │  └────────────┘  │    │                 │
└────────────────┘    │  ┌────────────┐  │    │                 │
                      │  │ OLED       │  │    │                 │
                      │  │ Display    │  │    │                 │
                      │  └────────────┘  │    │                 │
                      └──────────────────┘    └─────────────────┘
```

## Node 1: LilyGo LoRa32 + TF02-Pro LiDAR

### Physical Configuration

```
                    ┌──────────────────────────────┐
                    │   Weatherproof Enclosure      │
                    │                               │
                    │  ┌────────────────────────┐  │
                    │  │   LilyGo LoRa32        │  │
                    │  │                        │  │
                    │  │  ┌──────────────────┐  │  │
                    │  │  │  ESP32 MCU       │  │  │
                    │  │  │  + LoRa Module   │  │  │
                    │  │  └──────────────────┘  │  │
                    │  │                        │  │
                    │  │  ┌──────────────────┐  │  │
                    │  │  │  LiPo Battery    │  │  │
                    │  │  │  (2000mAh)       │  │  │
                    │  │  └──────────────────┘  │  │
                    │  └────────────────────────┘  │
                    │                               │
                    │  ┌────────────────────────┐  │
                    │  │   TF02-Pro LiDAR       │  │
                    │  │   (UART Interface)     │  │
                    │  │                        │  │
                    │  │   [VCC] [TX] [RX] [GND]│  │
                    │  └────────────────────────┘  │
                    │                               │
                    │  ┌────────────────────────┐  │
                    │  │   LoRa Antenna         │  │
                    │  └────────────────────────┘  │
                    └──────────────────────────────┘
                              │
                              │ 2-4m above water
                              ▼
                    ┌─────────────────┐
                    │   River Surface  │
                    └─────────────────┘
```

### Wiring Details

```
TF02-Pro Sensor          LilyGo LoRa32
═══════════════          ═════════════
VCC (Red)        ──────> 5V
TX  (Blue)       ──────> GPIO 16 (Serial2 RX)
RX  (Yellow)     ──────> GPIO 17 (Serial2 TX)
GND (Black)      ──────> GND

Battery          ──────> JST-PH 2.0 Connector
Antenna          ──────> U.FL/SMA Connector
```

### Power Flow

```
LiPo Battery (3.7V, 2000mAh)
    │
    ├──> LilyGo Board (3.3V/5V regulation)
    │    │
    │    ├──> ESP32 MCU
    │    ├──> LoRa Module (SX1276/8)
    │    └──> TF02-Pro Sensor (5V)
    │
    └──> Battery Life: 6-12 months
         (with 15min intervals, deep sleep)
```

## Node 2: Heltec LoRa32 V2 + JSN-SR04T Ultrasonic

### Physical Configuration

```
                    ┌──────────────────────────────┐
                    │   Weatherproof Enclosure      │
                    │                               │
                    │  ┌────────────────────────┐  │
                    │  │   Heltec LoRa32 V2      │  │
                    │  │                        │  │
                    │  │  ┌──────────────────┐  │  │
                    │  │  │  ESP32 MCU       │  │  │
                    │  │  │  + LoRa Module   │  │  │
                    │  │  └──────────────────┘  │  │
                    │  │                        │  │
                    │  │  ┌──────────────────┐  │  │
                    │  │  │  OLED Display    │  │  │
                    │  │  │  (128x64)        │  │  │
                    │  │  └──────────────────┘  │  │
                    │  └────────────────────────┘  │
                    │                               │
                    │  ┌────────────────────────┐  │
                    │  │   External Battery     │  │
                    │  │   Board (2000mAh)       │  │
                    │  └────────────────────────┘  │
                    │                               │
                    │  ┌────────────────────────┐  │
                    │  │   JSN-SR04T Board      │  │
                    │  │   (GPIO Interface)     │  │
                    │  │                        │  │
                    │  │   [5V] [TRIG] [ECHO]   │  │
                    │  │   [GND]                │  │
                    │  └────────────────────────┘  │
                    │                               │
                    │  ┌────────────────────────┐  │
                    │  │   Waterproof Probe     │  │
                    │  │   (2.5m cable, IP67)   │  │
                    │  └────────────────────────┘  │
                    │                               │
                    │  ┌────────────────────────┐  │
                    │  │   LoRa Antenna        │  │
                    │  └────────────────────────┘  │
                    └──────────────────────────────┘
                              │
                              │ 2-4m above water
                              ▼
                    ┌─────────────────┐
                    │   River Surface  │
                    └─────────────────┘
```

### Wiring Details

```
JSN-SR04T Sensor         Heltec LoRa32 V2
════════════════         ════════════════
5V  (Red)        ──────> 5V
TRIG (Orange)    ──────> GPIO 13
ECHO (Yellow)    ──────> GPIO 12
GND (Black)      ──────> GND

External Battery ──────> Battery Board Input
Battery Board    ──────> Heltec 5V Input
Antenna          ──────> U.FL/SMA Connector
```

## Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Data Flow Diagram                        │
└─────────────────────────────────────────────────────────────┘

Node 1 (LilyGo + TF02-Pro)          Node 2 (Heltec + JSN-SR04T)
──────────────────────────          ───────────────────────────

Wake from Deep Sleep                Wake from Deep Sleep
    │                                    │
    ├─> Initialize TF02-Pro             ├─> Initialize JSN-SR04T
    │   (UART Serial2)                   │   (GPIO Trigger/Echo)
    │                                    │
    ├─> Read 5 measurements              ├─> Read 5 measurements
    │   (with averaging)                 │   (with averaging)
    │                                    │
    ├─> Apply temperature                ├─> Calculate distance
    │   compensation                     │   (speed of sound)
    │   (Mohammed 2019, Tawalbeh 2023)   │
    │                                    │
    ├─> Format payload                   ├─> Format payload
    │   (8 bytes)                         │   (8 bytes)
    │                                    │
    └─> Transmit via LoRaWAN             └─> Transmit via LoRaWAN
            │                                    │
            └────────────┬───────────────────────┘
                         │
                         ▼
            ┌────────────────────────┐
            │   Wisgate Edge Pro     │
            │   LoRaWAN Gateway      │
            └────────────┬───────────┘
                         │
                         ▼
            ┌────────────────────────┐
            │   The Things Network   │
            │   (TTN)                │
            └────────────┬───────────┘
                         │
            ┌────────────┴────────────┐
            │                         │
            ▼                         ▼
    ┌──────────────┐         ┌──────────────┐
    │  MQTT        │         │  TTN Console │
    │  Broker      │         │  Web UI     │
    └──────────────┘         └──────────────┘
            │                         │
            └────────────┬─────────────┘
                         │
                         ▼
            ┌────────────────────────┐
            │   Data Storage &       │
            │   Analysis             │
            └────────────────────────┘
```

## Power Management Flow

```
┌─────────────────────────────────────────────────────────────┐
│              Power Management Cycle (15 minutes)            │
└─────────────────────────────────────────────────────────────┘

Time: 0:00
    │
    ├─> ESP32 wakes from deep sleep (~10µA)
    │
    ├─> Initialize sensors (~100mA)
    │   Duration: ~500ms
    │
    ├─> Read sensor data (~100-200mA)
    │   Duration: ~2-5 seconds
    │   (5 readings with delays)
    │
    ├─> Process data (~80mA)
    │   Duration: ~100ms
    │
    ├─> Transmit via LoRa (~120mA TX)
    │   Duration: ~1-2 seconds
    │
    ├─> Wait for RX window (~15mA RX)
    │   Duration: ~1-2 seconds
    │
    └─> Enter deep sleep (~10µA)
        Duration: ~15 minutes

Total Active Time: ~5-10 seconds
Total Sleep Time: ~900 seconds
Duty Cycle: ~0.5-1%
Battery Life: 6-12 months (2000mAh LiPo)
```

## Network Topology

```
                    Star Topology (LoRaWAN)

                         Gateway
                    (Wisgate Edge Pro)
                            │
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        │                   │                   │
    Node 1              Node 2              Node N
  (LilyGo)           (Heltec)            (Future)
  TF02-Pro          JSN-SR04T
    │                   │
    └───────────────────┘
        Direct communication
        (no mesh, no routing)
```

## Deployment Site Configuration

```
                    River Monitoring Site

    ┌─────────────────────────────────────────────┐
    │                                             │
    │  ┌──────────────┐      ┌──────────────┐   │
    │  │   Node 1     │      │   Node 2     │   │
    │  │  (LilyGo)    │      │  (Heltec)    │   │
    │  │              │      │              │   │
    │  │  TF02-Pro    │      │  JSN-SR04T   │   │
    │  │  LiDAR       │      │  Ultrasonic  │   │
    │  │              │      │              │   │
    │  │  2-4m above  │      │  2-4m above  │   │
    │  │  water level │      │  water level │   │
    │  └──────┬───────┘      └──────┬───────┘   │
    │         │                     │            │
    │         │                     │            │
    │         └──────────┬──────────┘            │
    │                    │                       │
    │              ┌─────▼─────┐                 │
    │              │   River    │                 │
    │              │   Surface  │                 │
    │              └────────────┘                 │
    │                                             │
    │  ┌─────────────────────────────────────┐   │
    │  │   Gateway (Wisgate Edge Pro)        │   │
    │  │   - Line of sight to nodes          │   │
    │  │   - Internet connection             │   │
    │  │   - Power supply                    │   │
    │  └─────────────────────────────────────┘   │
    │                                             │
    └─────────────────────────────────────────────┘
```

## Component Specifications Summary

### Node 1 (LilyGo + TF02-Pro)
- **MCU:** ESP32 dual-core
- **LoRa:** SX1276/8 (915MHz AU915)
- **Sensor:** TF02-Pro LiDAR (UART, 22m range)
- **Power:** Direct LiPo (2000mAh)
- **Interface:** UART (Serial2, GPIO 16/17)
- **Battery Life:** 6-12 months

### Node 2 (Heltec + JSN-SR04T)
- **MCU:** ESP32-WROOM-32
- **LoRa:** SX1276/8 (915MHz AU915)
- **Sensor:** JSN-SR04T Ultrasonic (GPIO, 4.5m range)
- **Power:** External battery board (2000mAh)
- **Interface:** GPIO (TRIG: GPIO 13, ECHO: GPIO 12)
- **Display:** OLED 128x64
- **Battery Life:** 6-12 months

### Gateway
- **Model:** Wisgate Edge Pro
- **Frequency:** AU915 (915MHz)
- **Network:** The Things Network (TTN)
- **Connectivity:** Internet (Ethernet/WiFi)
- **Range:** Up to 15km (line of sight)

## Data Transmission Details

### Payload Structure (8 bytes)
```
Byte 0:  Sensor Type (1=TF02-Pro, 3=JSN-SR04T)
Byte 1-2: Distance (mm, little-endian)
Byte 3-4: Signal Strength (LiDAR flux or 0)
Byte 5:   Temperature (°C, signed)
Byte 6:   Battery Level (0-100%)
Byte 7:   Reading Count (valid readings averaged)
```

### Transmission Schedule
- **Interval:** 15 minutes (900 seconds)
- **Frequency:** 96 readings per day
- **Data Rate:** SF7 (AU915)
- **TX Power:** 14 dBm
- **Success Rate Target:** >95%

## Environmental Considerations

### Deployment Requirements
1. **Mounting Height:** 2-4 meters above expected water level
2. **Enclosure:** Weatherproof (IP65/IP67)
3. **Antenna:** Vertical orientation, line-of-sight to gateway
4. **Power:** Battery with optional solar panel for extended operation
5. **Access:** For maintenance and battery replacement

### Environmental Factors
- **Temperature:** -10°C to +50°C (typical)
- **Humidity:** Protected by enclosure
- **Water:** Sensors positioned above water surface
- **Wind:** Secure mounting required
- **Sunlight:** Consider for battery temperature

## Maintenance Schedule

- **Battery Replacement:** Every 6-12 months
- **Sensor Calibration:** Quarterly verification
- **Enclosure Inspection:** Monthly
- **Data Validation:** Continuous monitoring
- **Firmware Updates:** As needed (OTA possible)
