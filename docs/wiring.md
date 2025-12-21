# Wiring Diagrams

This document provides wiring diagrams for connecting the Benewake TF-Luna sensor to both Heltec and LilyGo LoRa32 boards.

## TF-Luna Sensor Pinout

The TF-Luna sensor has 4 pins:
- **VCC** - Power supply (3.3V)
- **GND** - Ground
- **SDA** - I2C Data line
- **SCL** - I2C Clock line

## Heltec WiFi LoRa 32 V2 Wiring

```
TF-Luna Sensor          Heltec LoRa32 V2
─────────────────       ─────────────────
VCC              ──────> 3.3V
GND              ──────> GND
SDA              ──────> GPIO 21 (SDA)
SCL              ──────> GPIO 22 (SCL)
```

### Pin Reference
- **3.3V:** Power pin (any 3.3V pin)
- **GND:** Ground pin (any GND pin)
- **GPIO 21:** I2C SDA (default ESP32 I2C data line)
- **GPIO 22:** I2C SCL (default ESP32 I2C clock line)

## LilyGo LoRa32 Wiring

```
TF-Luna Sensor          LilyGo LoRa32
─────────────────       ─────────────────
VCC              ──────> 3.3V
GND              ──────> GND
SDA              ──────> GPIO 21 (SDA)
SCL              ──────> GPIO 22 (SCL)
```

### Pin Reference
- **3.3V:** Power pin (any 3.3V pin)
- **GND:** Ground pin (any GND pin)
- **GPIO 21:** I2C SDA (default ESP32 I2C data line)
- **GPIO 22:** I2C SCL (default ESP32 I2C clock line)

## Power Considerations

### Power Supply Options

1. **USB Power:**
   - Connect via USB cable
   - Provides 5V, regulated to 3.3V on board
   - Good for development and testing

2. **Battery Power:**
   - Heltec V2: Has built-in battery management
   - LilyGo: Check your model for battery support
   - Use appropriate battery (LiPo recommended)
   - Monitor battery voltage in code

3. **External Power:**
   - Use regulated 3.3V or 5V supply
   - Ensure adequate current capacity (500mA+ recommended)

### Power Consumption

- **ESP32:** ~80-240mA (active), ~10µA (deep sleep)
- **LoRa Module:** ~120mA (TX), ~15mA (RX), ~1µA (sleep)
- **TF-Luna:** ~100mA (typical)

**Total Active:** ~300-450mA  
**Total Sleep:** ~11µA (with deep sleep enabled)

## I2C Pull-up Resistors

Most ESP32 boards have built-in pull-up resistors on I2C lines. If you experience I2C communication issues:

1. Check if pull-ups are enabled in code
2. Add external 4.7kΩ pull-up resistors if needed:
   - Connect 4.7kΩ resistor from SDA to 3.3V
   - Connect 4.7kΩ resistor from SCL to 3.3V

## Antenna Connection

### LoRa Antenna
- Both boards have integrated LoRa modules
- Connect appropriate antenna to the U.FL/SMA connector
- Use antenna matching your frequency band:
  - EU868: 868 MHz antenna
  - US915: 915 MHz antenna
  - AS923: 923 MHz antenna

### Antenna Placement
- Place antenna vertically for best performance
- Keep away from metal objects
- Maintain line of sight to gateway when possible

## Troubleshooting Wiring

### Sensor Not Detected
1. **Check Power:**
   - Verify 3.3V at TF-Luna VCC pin
   - Check GND connection
   - Measure voltage with multimeter

2. **Check I2C:**
   - Verify SDA/SCL connections
   - Check for loose connections
   - Test with I2C scanner sketch

3. **Check Address:**
   - Default TF-Luna address: 0x10
   - Use I2C scanner to verify address

### Communication Errors
1. **I2C Bus Issues:**
   - Only one master on I2C bus
   - Check for short circuits
   - Verify pull-up resistors

2. **Power Issues:**
   - Insufficient power can cause communication failures
   - Check power supply current capacity
   - Measure voltage under load

### Range Issues
1. **Antenna:**
   - Verify antenna is connected
   - Check antenna type matches frequency
   - Ensure antenna is not damaged

2. **Signal Quality:**
   - Check RSSI values in serial output
   - Verify gateway is receiving
   - Check for interference

## Safety Notes

- Always power off before making connections
- Double-check pin connections before powering on
- Use appropriate wire gauge (22-24 AWG recommended)
- Secure connections to prevent shorts
- Protect from moisture if used outdoors

## Additional Resources

- [TF-Luna Datasheet](https://www.benewake.com/product-detail/tf-luna)
- [ESP32 Pin Reference](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
- [I2C Protocol Guide](https://www.i2c-bus.org/)

