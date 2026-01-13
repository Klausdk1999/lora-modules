# LilyGo T-Beam AXP2101 v1.2 - River Level Monitoring Node

This folder contains the firmware for the LilyGo T-Beam AXP2101 v1.2 board (FCC ID: 2asye-t-beam) configured as a river level monitoring sensor node.

## 🔧 Hardware

- **Board:** LilyGo T-Beam AXP2101 v1.2 (FCC ID: 2asye-t-beam)
- **MCU:** ESP32
- **LoRa Module:** SX1276/8 (integrated)
- **Power Management:** AXP2101 PMU (integrated)
- **Sensor:** TF02-Pro LiDAR sensor (UART interface, 22m range)
- **Note:** TF02-Pro has internal temperature sensor, no external temperature sensor needed

## ⚠️ Important Note

This firmware is configured for the **LilyGo T-Beam AXP2101 v1.2** board. The LoRaWAN configuration has been carefully tested and is working correctly. **Do not modify the LoRaWAN-related code** without understanding the implications.

If you have a different LilyGo/TTGO board variant, you may need to adjust the pin mappings in `src/main.cpp` based on your specific model.

## 📡 Pin Configuration

### TF02-Pro LiDAR Sensor (UART) - WORKING CONFIGURATION
- **TF02-Pro TX:** GPIO 13 (Serial2 RX pin)
- **TF02-Pro RX:** GPIO 14 (Serial2 TX pin)
- **VCC:** 5V (or 3.3V if sensor supports it)
- **GND:** GND
- **Note:** 
  - Extended range LiDAR sensor (22m) validated for larger rivers (Santana et al. 2024)
  - GPIO 14/13 are used to avoid conflict with USB Serial (GPIO 1/3) and GPS (GPIO 12/34)
  - **IMPORTANT:** Disconnect RX/TX cables before uploading code to avoid upload failures

### LoRa Module Pin Configuration (T-Beam AXP2101 v1.2)

**Current Configuration (T-Beam AXP2101 v1.2):**
- **NSS (Chip Select):** GPIO 18
- **RST (Reset):** GPIO 23
- **DIO0 (TX/RX Complete):** GPIO 26
- **DIO1 (RX Timeout):** GPIO 33
- **DIO2 (FSK Mode):** GPIO 32

**Note:** These pins are hardcoded in the `lmic_pins` structure and are critical for proper LoRaWAN operation. Do not change without verifying your board's schematic.

#### Other Board Variants (for reference):
- **LilyGo LoRa32 V1.6:**
  - NSS: GPIO 18
  - RST: GPIO 14 (different from T-Beam)
  - DIO0: GPIO 26
  - DIO1: GPIO 33
  - DIO2: GPIO 32

### Battery Monitoring (T-Beam AXP2101 v1.2)
- **Power Management:** AXP2101 PMU (integrated)
- **Note:** T-Beam uses AXP2101 PMU for battery management, not a simple ADC pin
- The current code uses GPIO 35 as a placeholder - battery monitoring may need to be implemented via AXP2101 I2C interface for accurate readings

## ⚙️ Configuration

Before uploading, edit `src/main.cpp` and configure:

1. **LoRaWAN Credentials:**
   ```cpp
   static const u1_t PROGMEM APPEUI[8] = { ... };
   static const u1_t PROGMEM DEVEUI[8] = { ... };
   static const u1_t PROGMEM APPKEY[16] = { ... };
   ```

2. **Sensor Test Mode (Optional):**
   ```cpp
   #define SENSOR_TEST_MODE  false  // Set to true to disable LoRaWAN and test sensor only
   ```
   When `SENSOR_TEST_MODE` is `true`, the code will:
   - Skip all LoRaWAN initialization
   - Run continuous sensor readings via Serial
   - Print detailed sensor data every 5 seconds
   - Useful for testing sensor without LoRaWAN gateway

3. **Pin Mappings:**
   - **DO NOT** change `lmic_pins` structure unless you have a different board variant
   - Current pins are configured for T-Beam AXP2101 v1.2 and are working correctly

4. **Timing:**
   ```cpp
   #define TX_INTERVAL_SECONDS  900  // 15 minutes (900 seconds)
   ```

5. **Region Settings (AU915 - Australia/Brazil):**
   - Frequency band: AU915 (configured in `platformio.ini` with `-DCFG_au915=1`)
   - Sub-band: 1 (channels 8-15) - configured in `LMIC_selectSubBand(1)`
   - Data Rate: SF7 (Spreading Factor 7) - optimal balance of range and power
   - Power: 14 dBm - configured in `LMIC_setDrTxpow(DR_SF7, 14)`

## 📡 LoRaWAN Configuration Documentation

### Working Configuration (DO NOT MODIFY)

The LoRaWAN configuration in this firmware has been carefully tested and is working correctly. Key settings:

- **Frequency Band:** AU915 (915 MHz ISM band for Australia/Brazil)
- **Sub-band:** 1 (channels 8-15) - selected to avoid interference
- **Data Rate:** SF7 (Spreading Factor 7) - optimal for power consumption
- **Power Level:** 14 dBm - good range with reasonable power consumption
- **ADR (Adaptive Data Rate):** Disabled - maintains fixed SF7 for predictable power consumption
- **Link Check:** Disabled - reduces overhead
- **Class:** A (uplink-only, minimal power consumption)

### PlatformIO Build Flags

Critical build flags in `platformio.ini` that are required for proper operation:

- `-DCFG_au915=1` - Enables AU915 frequency band
- `-DCFG_sx1276_radio=1` - Specifies SX1276/8 radio module
- `-DARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS` - Suppresses config header requirement
- `-Dhal_init=lmic_hal_init` - Renames hal_init to avoid ESP-IDF conflict
- `-DDISABLE_PING=1` - Disables ping (not needed for Class A)
- `-DDISABLE_BEACONS=1` - Disables beacons (not needed for Class A)

See `platformio.ini` for detailed comments on each flag.

### Pin Configuration

The LoRa module pins are configured in the `lmic_pins` structure:
- **NSS (GPIO 18):** SPI chip select
- **RST (GPIO 23):** Hardware reset
- **DIO0 (GPIO 26):** TX/RX complete interrupt
- **DIO1 (GPIO 33):** RX timeout interrupt
- **DIO2 (GPIO 32):** FSK mode (not used)

These pins are specific to the T-Beam AXP2101 v1.2 and should not be changed.

## 🔍 Identifying Your Board

To identify your specific LilyGo model:
1. Check the board label/silkscreen
2. Look for model number (e.g., "T-Beam AXP2101 v1.2", "LoRa32 V1.6")
3. Check the FCC ID (T-Beam AXP2101 v1.2: 2asye-t-beam)
4. Check the GitHub repository: [LilyGo T-Beam](https://github.com/Xinyuan-LilyGO/LilyGO-T-Beam)
5. Compare pinout with your board's schematic

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

### Normal Mode (SENSOR_TEST_MODE = false):
- Sensor readings (distance, signal strength, temperature)
- Temperature compensation values
- Statistical filtering results (median, outliers removed)
- LoRaWAN join status
- Transmission confirmations (RSSI, SNR)
- Error messages

### Test Mode (SENSOR_TEST_MODE = true):
- Continuous sensor readings every 5 seconds
- Detailed reading information (raw, compensated, filtered)
- No LoRaWAN output (gateway not needed)

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
- **Check UART wiring:** TF02-Pro TX -> GPIO 13, TF02-Pro RX -> GPIO 14
- **Verify power:** TF02-Pro VCC -> 5V, GND -> GND
- **Check sensor LED:** Should be on when powered
- **Try swapping TX/RX:** May be inverted
- **Disconnect cables before upload:** RX/TX cables can interfere with upload process

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








