# Heltec WiFi LoRa 32 V2 - River Level Monitoring Node

This folder contains the firmware for the Heltec WiFi LoRa 32 V2 board configured as a river level monitoring sensor node.

## 🔧 Hardware

- **Board:** Heltec WiFi LoRa 32 V2
- **MCU:** ESP32
- **LoRa Module:** SX1276/8
- **Sensor:** Benewake TF-Luna (I2C)

## 📡 Pin Configuration

### TF-Luna Sensor (I2C)
- **SDA:** GPIO 21 (default ESP32 I2C SDA)
- **SCL:** GPIO 22 (default ESP32 I2C SCL)
- **VCC:** 3.3V
- **GND:** GND

### LoRa Module (Integrated)
- **NSS:** GPIO 18
- **RST:** GPIO 14
- **DIO0:** GPIO 26
- **DIO1:** GPIO 33
- **DIO2:** GPIO 32

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
- Sensor readings (distance, flux, temperature)
- LoRaWAN join status
- Transmission confirmations
- Error messages

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

