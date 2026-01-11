# LilyGo LoRa32 - River Level Monitoring Node

This folder contains the firmware for the LilyGo LoRa32 board configured as a river level monitoring sensor node.

## 🔧 Hardware

- **Board:** LilyGo LoRa32 (various models)
- **MCU:** ESP32
- **LoRa Module:** SX1276/8 (integrated)
- **Sensor:** TF02-Pro LiDAR sensor (UART interface, 22m range)
- **Note:** TF02-Pro has internal temperature sensor, no external temperature sensor needed

## ⚠️ Important Note

LilyGo produces several LoRa32 variants with different pin configurations. You may need to adjust the pin mappings in `src/main.cpp` based on your specific model.

## 📡 Pin Configuration

### TF02-Pro LiDAR Sensor (UART)
- **TX:** GPIO 16 (RX pin for Serial2)
- **RX:** GPIO 17 (TX pin for Serial2)
- **VCC:** 5V (or 3.3V if sensor supports it)
- **GND:** GND
- **Note:** Extended range LiDAR sensor (22m) validated for larger rivers (Santana et al. 2024)

### LoRa Module Pin Variations

#### Common LilyGo T-Beam:
- **NSS:** GPIO 18
- **RST:** GPIO 23
- **DIO0:** GPIO 26
- **DIO1:** GPIO 33
- **DIO2:** GPIO 32

#### LilyGo LoRa32 V1.6:
- **NSS:** GPIO 18
- **RST:** GPIO 14
- **DIO0:** GPIO 26
- **DIO1:** GPIO 33
- **DIO2:** GPIO 32

### Battery Monitoring
- **Battery ADC:** GPIO 35 or 36 (varies by model)
- Check your board's schematic for the correct pin
- **Note:** Battery voltage divider may vary by board model

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
   char nodeId[16] = "lilygo_001";
   ```

3. **Pin Mappings:**
   - Adjust `lmic_pins` structure if your board uses different pins
   - Update battery ADC pin in `readSensor()` function

4. **Timing:**
   ```cpp
   const unsigned long SENSOR_READ_INTERVAL = 5000;  // 5 seconds
   const unsigned long LORA_TX_INTERVAL = 30000;     // 30 seconds
   ```

5. **Region Settings:**
   - Adjust `LMIC_setDrTxpow()` for your region
   - EU868: `LMIC_setDrTxpow(DR_SF7, 14);`
   - US915: Different settings required

## 🔍 Identifying Your Board

To identify your specific LilyGo model:
1. Check the board label/silkscreen
2. Look for model number (e.g., "T-Beam", "LoRa32 V1.6")
3. Check the GitHub repository: [LilyGo LoRa32](https://github.com/Xinyuan-LilyGO/LilyGO-T3-S3)
4. Compare pinout with your board's schematic

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
- Sensor readings (distance, signal strength, temperature)
- Temperature compensation values
- Statistical filtering results (median, outliers removed)
- LoRaWAN join status
- Transmission confirmations (RSSI, SNR)
- Error messages

## 🎯 Academic References

This implementation is based on validated academic research:

### Temperature Compensation
- **Mohammed et al. (2019)**: "Highly Accurate Water Level Measurement System" - Temperature compensation critical for sub-centimeter accuracy
- **Tawalbeh et al. (2023)**: "Evaluation of Ultrasonic Sensors" - LiDAR sensor accuracy strongly dependent on internal temperature
- **Formula**: 0.1% per °C deviation from 25°C reference temperature

### Data Filtering
- **Kabi et al. (2023)**: Statistical filtering required for raw sensor data in river environments (median filtering, outlier removal)

### Power Management
- **Casals et al. (2017)**: Deep sleep energy model validated for multi-year battery life (2400mAh battery = 6-year lifespan with 5-min intervals at SF7)
- **Bouguera et al. (2018)**: Discrete state energy decomposition (E_total = E_sleep + E_wu + E_meas + E_proc + E_tx + E_rx)
- **Ballerini et al. (2020)**: LoRaWAN consumes order of magnitude less energy than NB-IoT

### Network Performance
- **Mikhaylov et al. (2018)**: LoRaWAN supports up to 1000 devices per gateway at SF7
- **Casals et al. (2017)**: Using SF7 for optimal balance (lower ToA = lower energy consumption, increasing SF from 7 to 12 increases energy by ~40x)

### Extended Range
- **Santana et al. (2024)**: TF02-Pro LiDAR sensor (22m range) validated for larger rivers

## 🔋 Power Management

The code includes basic power management:
- Battery voltage monitoring (pin may need adjustment)
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
- Check pin mappings match your board

### No Data Transmission
- Wait for join to complete (check serial output)
- Verify transmission interval has elapsed
- Check LMIC event messages

### Wrong Pin Configuration
- Check your board's documentation
- Verify pin numbers match your hardware
- Test with a simple pin test sketch first

## 📚 Resources

- [LilyGo GitHub](https://github.com/Xinyuan-LilyGO)
- [LilyGo LoRa32 Documentation](https://github.com/Xinyuan-LilyGO/LilyGO-T3-S3)








