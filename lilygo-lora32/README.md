# LilyGo LoRa32 - River Level Monitoring Node

This folder contains the firmware for the LilyGo LoRa32 board configured as a river level monitoring sensor node.

## 🔧 Hardware

- **Board:** LilyGo LoRa32 (various models)
- **MCU:** ESP32
- **LoRa Module:** SX1276/8
- **Sensor:** Benewake TF-Luna (I2C)

## ⚠️ Important Note

LilyGo produces several LoRa32 variants with different pin configurations. You may need to adjust the pin mappings in `src/main.cpp` based on your specific model.

## 📡 Pin Configuration

### TF-Luna Sensor (I2C)
- **SDA:** GPIO 21 (default ESP32 I2C SDA)
- **SCL:** GPIO 22 (default ESP32 I2C SCL)
- **VCC:** 3.3V
- **GND:** GND

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
- Sensor readings (distance, flux, temperature)
- LoRaWAN join status
- Transmission confirmations
- Error messages

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




