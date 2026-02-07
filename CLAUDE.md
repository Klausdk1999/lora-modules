# LoRa River Level Monitoring - Project Context

## Overview
Master's thesis project by Klaus Dieter Kupper (UNIVALI, Brazil) - "Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring". Develops a complete WSN (Wireless Sensor Network) for real-time river level monitoring using LoRaWAN.

## Repository Structure
- `lilygo-tfnova-ultrasonic/` - LilyGo T-Beam V1.2 node (TF-Nova LiDAR + AJ-SR04M Ultrasonic + DHT11)
- `heltec-lora32-v2/` - Heltec WiFi LoRa 32 V2 node (JSN-SR04T Ultrasonic + DHT11)
- `heltec-tf02pro/` - Heltec node variant with TF02-Pro LiDAR
- `lilygo-lora32/` - LilyGo LoRa32 node variant
- `lib/sensors/` - Shared sensor driver libraries (SensorBase, TFNova, AJSR04M, HCSR04, JSNSR04T)
- `backend/` - Backend data processing
- `docs/` - Documentation, architecture diagrams, research summaries

## Hardware Nodes (Validation Deployment)

### Node 1: LilyGo T-Beam V1.2 (`lilygo-tfnova-ultrasonic`)
- **Board:** ESP32 + SX1276 LoRa + AXP2101 PMU
- **Sensors:** TF-Nova LiDAR (0.1-12m, UART), AJ-SR04M Ultrasonic (0.2-8m, IP67), DHT11
- **LoRa pins:** NSS=18, RST=23, DIO={26,33,32}
- **Session persistence:** RTC memory (survives deep sleep, ABP after first OTAA join)
- **Payload:** 16 bytes packed struct (sensorFlags, distances, strength, temp, humidity, battery)

### Node 2: Heltec WiFi LoRa 32 V2 (`heltec-lora32-v2`)
- **Board:** ESP32 + SX1276 LoRa + OLED 128x64
- **Sensors:** JSN-SR04T Ultrasonic (25-450cm, IP67), DHT11
- **LoRa pins:** NSS=18, RST=14, DIO={26,35,34}
- **No session persistence** - full OTAA rejoin on every deep sleep wake
- **Payload:** 8 bytes packed struct (sensorType, distanceMm, signalStrength, temp, battery, readingCount)

## Technical Configuration
- **LoRaWAN:** AU915, Sub-band 1, Class A, OTAA, SF7, 14 dBm
- **Network:** The Things Network (TTN)
- **Deep sleep:** ESP32 RTC timer wakeup (~10-150 uA)
- **Sensor averaging:** 10 readings per sensor, 100ms between readings, median + outlier filtering (20% threshold)
- **Adaptive sleep:** 30min/20min/10min/5min/1min based on change rate between readings

## Adaptive Sleep Algorithm
Compares current distance reading with previous (stored in RTC memory). Change thresholds (mm):
- <= 2mm change -> 30 min (very slow, static water)
- <= 5mm change -> 20 min (slow)
- <= 15mm change -> 10 min (moderate)
- <= 50mm change -> 5 min (fast)
- > 50mm change -> 1 min (rapid, potential flood event)
- Default (first boot/sensor error): 10 min

## Build System
- PlatformIO (each node is a separate PlatformIO project)
- Build from each project directory: `cd <project-dir> && platformio run`
- Upload: `platformio run --target upload`
- Shared libraries in `lib/sensors/` are symlinked/referenced via `lib_extra_dirs` in platformio.ini

## Key Academic References
- Mohammed et al. 2019, Tawalbeh et al. 2023: Ultrasonic temperature compensation (v = 331.3 + 0.606*T)
- Kabi et al. 2023: Statistical filtering for river environment noise
- Casals et al. 2017, Bouguera et al. 2018: Energy models for ESP32 deep sleep
- Ballerini et al. 2020: LoRaWAN vs NB-IoT energy comparison

## Important Notes
- LoRaWAN keys (DevEUI, AppKey) are embedded in source - handle with care
- DHT11 temperature is used for ultrasonic speed-of-sound compensation (critical for accuracy)
- TF-Nova signal strength indicates reading quality (<100 poor, >1000 good)
- Battery monitoring: AXP2101 PMU on T-Beam, ADC voltage divider (GPIO 37) on Heltec
- Thesis document is in separate repo: `Artigo-mestrado/` (LaTeX, ABNT format)
