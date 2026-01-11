# Heltec WiFi LoRa 32 V2 - River Level Monitoring Node

This folder contains the firmware for the Heltec WiFi LoRa 32 V2 board configured as a river level monitoring sensor node.

## 🔧 Hardware

- **Board:** Heltec WiFi LoRa 32 V2
- **MCU:** ESP32
- **LoRa Module:** SX1276/8 (integrated)
- **Sensor:** JSN-SR04T Ultrasonic sensor (waterproof, 25-450cm range)
- **Temperature Sensor:** DHT11 (for ultrasonic speed-of-sound compensation)
- **Display:** OLED 128x64 (integrated)

## 📡 Pin Configuration

### JSN-SR04T Ultrasonic Sensor
- **TRIG:** GPIO 13
- **ECHO:** GPIO 12
- **VCC:** 5V
- **GND:** GND
- **Note:** Waterproof ultrasonic sensor validated for river monitoring (Mohammadreza MasoudiMoghaddam et al. 2024, Panagopoulos et al. 2021)

### DHT11 Temperature Sensor (Required for ultrasonic compensation)
- **DATA:** GPIO 27 (or GPIO 25, avoiding conflicts)
- **VCC:** 3.3V or 5V (DHT11 supports both)
- **GND:** GND
- **Note:** Temperature sensor must be physically located near ultrasonic transducer for accurate air column density measurement (Mohammed et al. 2019, Tawalbeh et al. 2023)
- **Alternative:** BMP280 available as optional secondary sensor for more accurate readings

### LoRa Module (Integrated)
- **NSS:** GPIO 18
- **RST:** GPIO 14
- **DIO0:** GPIO 26
- **DIO1:** GPIO 35
- **DIO2:** GPIO 34

### Battery Monitoring
- **Battery ADC:** GPIO 37 (VBAT_SENSE)

## ⚙️ Configuration

Before uploading, edit `src/main.cpp` and configure:

1. **LoRaWAN Credentials:**
   ```cpp
   static const u1_t PROGMEM APPEUI[8] = { ... };
   static const u1_t PROGMEM DEVEUI[8] = { ... };
   static const u1_t PROGMEM APPKEY[16] = { ... };
   ```

2. **Node ID:**
   ```cpp
   char nodeId[16] = "heltec_001";
   ```

3. **Timing:**
   ```cpp
   const unsigned long SENSOR_READ_INTERVAL = 5000;  // 5 seconds
   const unsigned long LORA_TX_INTERVAL = 30000;     // 30 seconds
   ```

4. **Region Settings:**
   - Adjust `LMIC_setDrTxpow()` for your region
   - EU868: `LMIC_setDrTxpow(DR_SF7, 14);`
   - US915: Different settings required

## 🚀 Building and Uploading

```bash
# Install dependencies
pio lib install

# Build project
pio run

# Upload to board
pio run -t upload

# Monitor serial output
pio device monitor
```

## 📊 Serial Output

The node outputs debug information via Serial (115200 baud):
- Sensor readings (distance, temperature, battery level)
- Temperature compensation values
- Statistical filtering results (median, outliers removed)
- LoRaWAN join status
- Transmission confirmations (RSSI, SNR)
- Error messages

## 🎯 Academic References

This implementation is based on validated academic research:

### Temperature Compensation
- **Mohammed et al. (2019)**: "Highly Accurate Water Level Measurement System" - Temperature compensation critical for sub-centimeter accuracy
- **Tawalbeh et al. (2023)**: "Evaluation of Ultrasonic Sensors" - Diurnal temperature swings of 20°C cause several centimeters error without compensation
- **Formula**: v(T) = 331.3 + 0.606 * θ (speed-of-sound compensation)

### Data Filtering
- **Kabi et al. (2023)**: Statistical filtering required for raw sensor data in river environments (noise from turbulence and debris)

### Power Management
- **Casals et al. (2017)**: Deep sleep energy model validated for multi-year battery life
- **Bouguera et al. (2018)**: Discrete state energy decomposition
- **Ballerini et al. (2020)**: LoRaWAN consumes order of magnitude less energy than NB-IoT

### Network Performance
- **Mikhaylov et al. (2018)**: LoRaWAN supports up to 1000 devices per gateway at SF7
- **Casals et al. (2017)**: Using SF7 for optimal balance (lower ToA = lower energy consumption)

## 🔋 Power Management

The code includes basic power management:
- Battery voltage monitoring
- Configurable transmission intervals
- Deep sleep can be added for extended battery life

## 🐛 Troubleshooting

### Sensor Not Detected
- Check I2C wiring (SDA/SCL)
- Verify TF-Luna power (3.3V)
- Check I2C address (default: 0x10)

### LoRaWAN Join Failed
- Verify DevEUI, AppEUI, AppKey match gateway
- Check frequency band matches region
- Ensure gateway is powered and connected

### No Data Transmission
- Wait for join to complete (check serial output)
- Verify transmission interval has elapsed
- Check LMIC event messages








