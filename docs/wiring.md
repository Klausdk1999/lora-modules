# Sensor Wiring Diagrams for River Level Monitoring

This document provides complete wiring diagrams for connecting sensors to both Heltec and LilyGo LoRa32 boards.

## Testing Groups Configuration

### Group A: LilyGo T-Beam AXP2101 v1.2 - LiDAR Testing
- **Board:** LilyGo T-Beam AXP2101 v1.2
- **Power:** 18650 battery to T-Beam + separate 18650 to 5V boost for TF02-Pro
- **Sensor:** TF02-Pro LiDAR (UART) - Extended range for larger rivers

### Group B: Heltec WiFi LoRa 32 V2 - Ultrasonic Testing
- **Board:** Heltec WiFi LoRa 32 V2
- **Power:** External battery board
- **Sensor:** JSN-SR04T (waterproof outdoor) - For field deployment comparison

---

## TF02-Pro LiDAR Sensor (UART)

### Specifications
| Parameter | Value |
|-----------|-------|
| Range | 0.1m - 22m (indoor), 0.1m - 12m (outdoor) |
| Accuracy | ±1cm (0.1-6m), ±1% (6-22m) |
| Interface | UART (default) or I2C |
| Voltage | 5V (or 3.3V if supported) |
| Current | ~100mA typical |
| Baud Rate | 115200 (default) |
| Frame Rate | 1-1000Hz (default 100Hz, reduced to 10Hz for power saving) |

### TF02-Pro Pinout (4 pins)
```
┌─────────────────────────┐
│  TF02-Pro LiDAR Sensor  │
│  ┌───────────────────┐  │
│  │  LASER APERTURE   │  │
│  └───────────────────┘  │
│                         │
│   [VCC] [TX] [RX] [GND]
└─────────────────────────┘
      │     │    │    │
     5V  Data Data GND
```

### Wiring to LilyGo T-Beam AXP2101 v1.2 (UART - Serial2)

```
TF02-Pro Sensor            LilyGo T-Beam AXP2101 v1.2
═══════════════            ═════════════
     VCC   ─────────────────>  5V boost output (external)
     GND   ─────────────────>  GND
     TX    ─────────────────>  GPIO 13 (Serial2 RX)
     RX    ─────────────────>  GPIO 14 (Serial2 TX)
```

**Physical Connection Diagram:**
```
                    ┌──────────────────────────────────┐
   TF02-Pro         │   LilyGo T-Beam AXP2101 v1.2      │
 ┌─────────┐         │                                  │
 │   ┌─┐   │         │  ┌────────────────────────────┐  │
 │   └─┘   │         │  │  LoRa Antenna Connector    │  │
 │         │         │  └────────────────────────────┘  │
│  VCC ───┼─Red─────┼───> 5V boost output (external)   │
│  TX  ───┼─Blue────┼───> GPIO 13 (Serial2 RX)         │
│  RX  ───┼─Yellow──┼───> GPIO 14 (Serial2 TX)         │
 │  GND ───┼─Black───┼───> GND                          │
 └─────────┘         │                                  │
                     │        [USB-C Port]              │
                     └──────────────────────────────────┘
```

**Note:** TF02-Pro uses UART interface (Serial2) instead of I2C. This provides:
- Longer range (22m vs 8m for TF-Luna)
- Better for larger rivers (as per thesis requirements)
- Temperature compensation needed (Mohammed et al. 2019, Tawalbeh et al. 2023)

---

## TF-Luna LiDAR Sensor (I2C) - Alternative Option

### Specifications
| Parameter | Value |
|-----------|-------|
| Range | 0.2m - 8m |
| Accuracy | ±6cm (0.2-3m), ±2% (3-8m) |
| Interface | I2C (also supports UART) |
| Voltage | 3.3V - 5V |
| Current | ~100mA typical |
| I2C Address | 0x10 (default) |

### TF-Luna Pinout (4 pins)
```
┌─────────────────────────┐
│  TF-Luna LiDAR Sensor   │
│  ┌───────────────────┐  │
│  │  LASER APERTURE   │  │
│  └───────────────────┘  │
│                         │
│   [VCC] [SDA] [SCL] [GND]
└─────────────────────────┘
      │     │     │    │
     3.3V  Data Clock GND
```

### Wiring to LilyGo LoRa32 (I2C - Alternative if TF02-Pro unavailable)

```
TF-Luna Sensor              LilyGo LoRa32
═══════════════             ═════════════
     VCC   ─────────────────>  3.3V
     GND   ─────────────────>  GND
     SDA   ─────────────────>  GPIO 21 (I2C Data)
     SCL   ─────────────────>  GPIO 22 (I2C Clock)
```

**Note:** TF-Luna is a fallback option. TF02-Pro is preferred for its extended range (22m).

**Physical Connection Diagram:**
```
                    ┌──────────────────────────────────┐
   TF-Luna          │        LilyGo LoRa32             │
 ┌─────────┐        │                                  │
 │   ┌─┐   │        │  ┌────────────────────────────┐  │
 │   └─┘   │        │  │  LoRa Antenna Connector    │  │
 │         │        │  └────────────────────────────┘  │
 │  VCC ───┼─Red────┼───> 3.3V                         │
 │  SDA ───┼─Blue───┼───> GPIO 21                      │
 │  SCL ───┼─Yellow─┼───> GPIO 22                      │
 │  GND ───┼─Black──┼───> GND                          │
 └─────────┘        │                                  │
                    │        [USB-C Port]              │
                    └──────────────────────────────────┘
```

### Wiring to Heltec WiFi LoRa 32 V2 (I2C - Alternative)

```
TF-Luna Sensor              Heltec LoRa32 V2
═══════════════             ════════════════
     VCC   ─────────────────>  3.3V
     GND   ─────────────────>  GND
     SDA   ─────────────────>  GPIO 21 (I2C Data)
     SCL   ─────────────────>  GPIO 22 (I2C Clock)
```

**Note:** Heltec node uses JSN-SR04T ultrasonic sensor. TF-Luna shown for reference only.

---

## HC-SR04 Ultrasonic Sensor (GPIO)

### Specifications
| Parameter | Value |
|-----------|-------|
| Range | 2cm - 400cm |
| Accuracy | ±3mm |
| Measuring Angle | 15° |
| Interface | Digital GPIO (Trigger/Echo) |
| Voltage | 5V |
| Current | ~15mA |

### HC-SR04 Pinout (4 pins)
```
┌──────────────────────────┐
│     HC-SR04 Sensor       │
│   ┌────┐       ┌────┐    │
│   │ TX │       │ RX │    │
│   └────┘       └────┘    │
│                          │
│  [VCC] [TRIG] [ECHO] [GND]
└──────────────────────────┘
     │     │      │     │
    5V  Trigger  Echo  GND
```

### Wiring to Heltec WiFi LoRa 32 V2

**Primary Configuration (JSN-SR04T - Waterproof):**
```
JSN-SR04T Sensor            Heltec LoRa32 V2
════════════════            ════════════════
     5V    ─────────────────>  5V (or 3.3V works too)
     TRIG  ─────────────────>  GPIO 13
     ECHO  ─────────────────>  GPIO 12
     GND   ─────────────────>  GND
```

**Alternative Configuration (HC-SR04 - Indoor testing):**
```
HC-SR04 Sensor              Heltec LoRa32 V2
══════════════              ════════════════
     VCC   ─────────────────>  5V (from USB or battery)
     TRIG  ─────────────────>  GPIO 13
     ECHO  ─────────────────>  GPIO 12
     GND   ─────────────────>  GND
```

**⚠️ Important:** The HC-SR04 ECHO pin outputs 5V signals. The ESP32 GPIO pins are 5V tolerant on input, but for safety, you can use a voltage divider:

**Optional Voltage Divider for ECHO pin:**
```
HC-SR04 ECHO ────┬──── 1kΩ ────┬───> GPIO 12
                 │              │
                 └── 2kΩ ──────┴───> GND
```

**Physical Connection Diagram:**
```
                    ┌────────────────────────────────────┐
   HC-SR04          │     Heltec WiFi LoRa 32 V2         │
 ┌───────────┐      │  ┌─────────────────────────────┐   │
 │  ┌─┐ ┌─┐  │      │  │     OLED Display            │   │
 │  │T│ │R│  │      │  │     ┌───────────┐           │   │
 │  └─┘ └─┘  │      │  │     │  Status   │           │   │
 │           │      │  │     │  Info     │           │   │
 │  VCC ─────┼─Red──┼──┼───> 5V                          │
 │  TRIG ────┼─Org──┼──┼───> GPIO 13                     │
 │  ECHO ────┼─Yel──┼──┼───> GPIO 12                     │
 │  GND ─────┼─Blk──┼──┼───> GND                         │
 └───────────┘      │  │     └───────────┘           │   │
                    │  └─────────────────────────────┘   │
                    │                                    │
                    │         [USB Port]                 │
                    └────────────────────────────────────┘
```

---

## JSN-SR04T Waterproof Ultrasonic Sensor (GPIO)

### Specifications
| Parameter | Value |
|-----------|-------|
| Range | 25cm - 450cm |
| Blind Zone | <25cm |
| Accuracy | ±1cm |
| Waterproof | IP67 (probe only) |
| Interface | Digital GPIO (Trigger/Echo) |
| Voltage | 3.3V - 5V |
| Current | ~30mA |

### JSN-SR04T Pinout
```
         Waterproof Probe
              ┌───┐
              │   │ ← Ultrasonic transducer
              └─┬─┘
                │ Cable (~2.5m)
    ┌───────────┴────────────┐
    │    JSN-SR04T Board     │
    │                        │
    │  [5V] [TRIG] [ECHO] [GND]
    └────────────────────────┘
         │     │      │    │
        5V  Trigger Echo  GND
```

### Wiring to Heltec WiFi LoRa 32 V2

```
JSN-SR04T Sensor            Heltec LoRa32 V2
════════════════            ════════════════
     5V    ─────────────────>  5V (or 3.3V works too)
     TRIG  ─────────────────>  GPIO 13
     ECHO  ─────────────────>  GPIO 12
     GND   ─────────────────>  GND
```

**Note:** JSN-SR04T can operate at 3.3V, making it more compatible with ESP32 without voltage dividers.

---

## AJ-SR04M Long-Range Ultrasonic Sensor (GPIO)

### Specifications
| Parameter | Value |
|-----------|-------|
| Range | 20cm - 800cm (8 meters!) |
| Accuracy | ±0.5cm |
| Waterproof | IP67 (probe only) |
| Interface | Digital GPIO or Serial UART |
| Voltage | 3.3V - 5V |
| Current | ~20mA |

### Wiring (same as JSN-SR04T)
```
AJ-SR04M Sensor             Heltec or LilyGo
═══════════════             ════════════════
     VCC   ─────────────────>  3.3V or 5V
     TRIG  ─────────────────>  GPIO 13
     ECHO  ─────────────────>  GPIO 12
     GND   ─────────────────>  GND
```

---

## Pin Summary Table

### LilyGo LoRa32 Available Pins

| Pin | Function | Used By |
|-----|----------|---------|
| GPIO 13 | Serial2 RX | **TF02-Pro TX (UART)** |
| GPIO 14 | Serial2 TX | **TF02-Pro RX (UART)** |
| GPIO 21 | I2C SDA | Alternative: TF-Luna SDA (if using I2C) |
| GPIO 22 | I2C SCL | Alternative: TF-Luna SCL (if using I2C) |
| GPIO 12 | Digital I/O | Available |
| GPIO 13 | Digital I/O | Available |
| GPIO 18 | SPI NSS | **LoRa (Reserved)** |
| GPIO 23 | SPI/RST | **LoRa (Reserved)** |
| GPIO 26 | Digital | **LoRa DIO0 (Reserved)** |
| GPIO 33 | Digital | **LoRa DIO1 (Reserved)** |
| GPIO 32 | Digital | **LoRa DIO2 (Reserved)** |
| GPIO 35 | ADC | Battery monitoring (if available) |

### Heltec WiFi LoRa 32 V2 Available Pins

| Pin | Function | Used By |
|-----|----------|---------|
| GPIO 21 | I2C SDA | TF-Luna SDA (if using) |
| GPIO 22 | I2C SCL | TF-Luna SCL (if using) |
| GPIO 12 | Digital I/O | Ultrasonic ECHO |
| GPIO 13 | Digital I/O | Ultrasonic TRIG |
| GPIO 4 | OLED SDA | **OLED (Reserved)** |
| GPIO 15 | OLED SCL | **OLED (Reserved)** |
| GPIO 16 | OLED RST | **OLED (Reserved)** |
| GPIO 18 | SPI NSS | **LoRa (Reserved)** |
| GPIO 14 | SPI RST | **LoRa (Reserved)** |
| GPIO 26 | Digital | **LoRa DIO0 (Reserved)** |
| GPIO 35 | Digital | **LoRa DIO1 (Reserved)** |
| GPIO 34 | Digital | **LoRa DIO2 (Reserved)** |

---

## Power Configuration

### LilyGo T-Beam AXP2101 v1.2 with Two Batteries
```
┌─────────────────────────────────────────┐
│   LilyGo T-Beam AXP2101 v1.2            │
│  ┌─────────────────────────────────┐    │
│  │    LiPo Battery Connector       │    │
│  │         (JST-PH 2.0)            │    │
│  └───────────┬─────────────────────┘    │
│              │                          │
│    ┌─────────┴─────────┐                │
│    │   18650 Battery   │                │
│    │   (3.7V)          │                │
│    └───────────────────┘                │
│                                         │
│  ✓ Built-in charging via USB-C          │
│  ✓ Battery monitoring via AXP2101       │
└─────────────────────────────────────────┘
```

**External 5V for TF02-Pro (second 18650 + boost):**
```
18650 (3.7V) → 5V boost module → TF02-Pro VCC
GND (boost)  → GND (T-Beam + TF02-Pro)
```
**Note:** The T-Beam 5V pin is only active with USB. Use the external boost for battery-only operation.

### Heltec with External Battery Board
```
┌────────────────────────────────────────────┐
│         External Battery Setup             │
│                                            │
│  ┌──────────────┐    ┌────────────────┐    │
│  │ LiPo Battery │───>│ Battery Board  │    │
│  │  3.7V 2000mAh│    │ (with 5V boost)│    │
│  └──────────────┘    └───────┬────────┘    │
│                              │             │
│                    5V Output │             │
│                              ▼             │
│              ┌───────────────────────┐     │
│              │  Heltec LoRa32 V2     │     │
│              │  (5V input or USB)    │     │
│              └───────────────────────┘     │
└────────────────────────────────────────────┘
```

---

## Power Consumption Estimates

### LiDAR Node (LilyGo + TF02-Pro)
| Component | Active | Sleep |
|-----------|--------|-------|
| ESP32 | 80-240mA | ~10µA |
| LoRa TX | ~120mA | ~1µA |
| TF02-Pro | ~100mA | ~20mA (idle) |
| **Total Active** | **~350-450mA** | **~30µA** |
| **Battery Life** | **6-12 months** (with 2000mAh LiPo, 15min intervals) |

**Power Management:**
- Deep sleep between transmissions (15 minutes)
- Active time: ~5-10 seconds per cycle
- Frame rate reduced to 10Hz for power saving

### Ultrasonic Node (Heltec + JSN-SR04T)
| Component | Active | Sleep |
|-----------|--------|-------|
| ESP32 | 80-240mA | ~10µA |
| LoRa TX | ~120mA | ~1µA |
| OLED Display | ~20mA | ~0µA (powered off in sleep) |
| JSN-SR04T | ~30mA | ~2mA |
| **Total Active** | **~300-400mA** | **~13µA** |
| **Battery Life** | **6-12 months** (with 2000mAh battery, 15min intervals) |

**Power Management:**
- Deep sleep between transmissions (15 minutes)
- OLED display powered off during sleep
- Active time: ~5-10 seconds per cycle

---

## Troubleshooting

### TF02-Pro Not Detected
1. Check UART connections (TX→GPIO16, RX→GPIO17)
2. Verify 5V power supply (or 3.3V if sensor supports it)
3. Check baud rate is set to 115200
4. Verify Serial2 is initialized correctly
5. Check for proper frame synchronization (sensor outputs continuous frames)

### TF-Luna Not Detected (Alternative I2C sensor)
1. Check I2C connections (SDA→21, SCL→22)
2. Verify 3.3V power supply
3. Run I2C scanner to verify address (should be 0x10)
4. Check for pull-up resistors (usually built into ESP32)

### Ultrasonic Sensor Returns 0 or -1
1. Verify TRIG and ECHO pin connections
2. Check power supply (5V for HC-SR04)
3. Ensure no obstacles in the blind zone (<2cm for HC-SR04, <25cm for JSN-SR04T)
4. Test with longer timeout if measuring far distances

### LoRa Not Joining
1. Check antenna connection
2. Verify TTN credentials (APPEUI, DEVEUI, APPKEY)
3. Ensure gateway is in range
4. Check frequency plan matches your region (AU915 for Brazil)

---

## Quick Reference Wiring

### For TF02-Pro (UART - Primary):
```
VCC → 5V boost output | TX → GPIO 13 (Serial2 RX) | RX → GPIO 14 (Serial2 TX) | GND → GND
```

### For TF-Luna (I2C - Alternative):
```
VCC → 3.3V | SDA → GPIO 21 | SCL → GPIO 22 | GND → GND
```

### For Ultrasonic (GPIO):
```
VCC → 5V (or 3.3V for JSN-SR04T) | TRIG → GPIO 13 | ECHO → GPIO 12 | GND → GND
```
