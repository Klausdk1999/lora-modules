# Heltec WiFi LoRa 32 V2 - TF02-Pro + DHT11 Node Pinout

## Overview
This node uses a Heltec WiFi LoRa 32 V2 with:
- TF02-Pro LiDAR sensor (UART)
- DHT11 Temperature/Humidity sensor
- OLED display (built-in)

## Power Supply
- **5V External Battery** powers the TF02-Pro sensor
- TF02-Pro requires 5V-12V (300mA peak)
- DHT11 can work on 3.3V or 5V
- The Heltec's built-in USB or battery connector powers the ESP32 and LoRa module

## Wiring Diagram (ASCII)

```
                    +--------------------+
                    | Heltec WiFi LoRa   |
                    |     32 V2          |
                    +--------------------+
                    |    [OLED Display]  |
                    |    Built-in        |
                    +--------------------+
                    |                    |
    5V Battery -----| 5V      GND |------+---- GND (common)
                    |                    |
                    | GPIO 13 (RX) |-----+---- TF02-Pro TX (green)
                    | GPIO 17 (TX) |-----+---- TF02-Pro RX (white)
                    |                    |
                    | GPIO 25 (DATA)|----+---- DHT11 DATA
                    |                    |
                    | GPIO 37 (ADC) |----+---- Battery voltage divider (internal)
                    +--------------------+


    TF02-Pro LiDAR                       DHT11
    +-------------+                      +---------+
    | VCC (red)   |---5V                 | VCC     |---3.3V/5V
    | GND (black) |---GND                | GND     |---GND
    | TX (green)  |---GPIO 13            | DATA    |---GPIO 25
    | RX (white)  |---GPIO 17            +---------+
    +-------------+
```

## Pin Assignment Table

| Function          | GPIO | Direction | Connected To        | Notes                          |
|-------------------|------|-----------|---------------------|--------------------------------|
| **LoRa (SPI)**    |      |           |                     | Internal - DO NOT USE          |
| NSS               | 18   | Output    | LoRa SX1276         | Chip select                    |
| RST               | 14   | Output    | LoRa SX1276         | Reset                          |
| DIO0              | 26   | Input     | LoRa SX1276         | TX/RX complete interrupt       |
| DIO1              | 35   | Input     | LoRa SX1276         | RX timeout                     |
| DIO2              | 34   | Input     | LoRa SX1276         | FSK mode (unused)              |
| **OLED (I2C)**    |      |           |                     | Internal - Built-in display    |
| SCL               | 15   | Output    | SSD1306             | I2C clock                      |
| SDA               | 4    | Bidirect  | SSD1306             | I2C data                       |
| RST               | 16   | Output    | SSD1306             | Display reset                  |
| **TF02-Pro (UART)**|     |           |                     | Serial2                        |
| RX                | 13   | Input     | TF02-Pro TX         | Receives sensor data           |
| TX                | 17   | Output    | TF02-Pro RX         | Sends commands                 |
| **DHT11**         |      |           |                     | Single-wire protocol           |
| DATA              | 25   | Bidirect  | DHT11 DATA          | Temperature/humidity           |
| **RESERVED SPI**  |      |           |                     | DO NOT USE for sensors         |
| MOSI              | 27   | Output    | LoRa SX1276         | SPI Master Out - conflicts!    |
| **Battery ADC**   |      |           |                     | Internal                       |
| ADC               | 37   | Input     | Voltage divider     | Battery monitoring             |

## Sensor Specifications

### TF02-Pro LiDAR
- **Interface**: UART @ 115200 baud
- **Range**: 0.1m to 22m (indoor), 0.1m to 12m (outdoor)
- **Accuracy**: ±1cm (0.1-6m), ±1% (6-22m)
- **Power**: 5V-12V, ~300mA peak
- **Frame format**: [0x59][0x59][Dist_L][Dist_H][Str_L][Str_H][Temp_L][Temp_H][Checksum]
- **Built-in temperature sensor**: For distance compensation

### DHT11
- **Interface**: Single-wire digital
- **Temperature range**: 0-50°C, ±2°C accuracy
- **Humidity range**: 20-80%, ±5% accuracy
- **Power**: 3.3V-5V, ~2.5mA

## Power Considerations

1. **TF02-Pro requires 5V-12V** (5V recommended)
   - Peak current: ~300mA
   - Use external 5V battery or boost converter
2. **Total current draw** (peak):
   - TF02-Pro: ~300mA
   - DHT11: ~2.5mA
   - ESP32 + OLED active: ~120mA
   - **Total**: ~420mA peak
3. **Deep sleep current**: ~10µA (ESP32 only, OLED off)

## Available GPIO Pins (Not Used in This Project)

These pins are available for expansion:
- GPIO 0 (has boot mode function - use with caution)
- GPIO 2 (has boot mode function - use with caution)
- GPIO 12 (strapping pin - avoid for inputs at boot)
- GPIO 32
- GPIO 33
- GPIO 36 (input only)
- GPIO 39 (input only)

**WARNING: GPIO 27 is SPI MOSI - DO NOT USE for sensors!**
Using GPIO 27 for sensors will cause LoRa transmission failures.

## Troubleshooting

### TF02-Pro not responding
1. Check 5V power supply is delivering enough current (>400mA)
2. Verify TX/RX wiring (sensor TX -> ESP RX, sensor RX -> ESP TX)
3. Check baud rate is 115200
4. Wait 1 second after power-on before reading
5. Try power cycling the sensor

### DHT11 NaN readings
1. Wait 2 seconds after power-on before reading
2. Add 10K pullup resistor on DATA line
3. Ensure minimum 1 second between readings
4. Verify VCC is connected (3.3V or 5V)

### OLED not displaying
1. Check that GPIO 16 (RST) is not held low
2. Ensure I2C is not blocked by other code
3. Verify u8g2 library is installed

### Battery reading incorrect
1. Heltec V2 has internal voltage divider on GPIO 37
2. Calibrate the ADC reading if needed
3. Check if battery is connected via JST connector

## LoRa Payload Format (12 bytes)

```
Byte  Field              Type     Description
0     sensorType         uint8    1 = TF02-Pro, 0xFF = error
1-2   distanceMm         int16    Distance in mm (-1 = error)
3-4   signalStrength     int16    TF02-Pro signal strength (flux)
5     sensorTemp         int8     TF02-Pro internal temperature (Celsius)
6     ambientTemp        int8     DHT11 temperature (Celsius)
7     humidity           uint8    DHT11 humidity (%)
8     batteryPercent     uint8    Battery level (0-100%)
9     readingCount       uint8    Number of valid readings (1-2)
```

## TTN Payload Decoder

```javascript
function decodeUplink(input) {
  var bytes = input.bytes;

  var sensorType = bytes[0];
  var distanceMm = (bytes[2] << 8) | bytes[1];
  if (distanceMm > 32767) distanceMm -= 65536;  // Convert to signed

  var signalStrength = (bytes[4] << 8) | bytes[3];
  if (signalStrength > 32767) signalStrength -= 65536;

  var sensorTemp = bytes[5];
  if (sensorTemp > 127) sensorTemp -= 256;

  var ambientTemp = bytes[6];
  if (ambientTemp > 127) ambientTemp -= 256;

  var humidity = bytes[7];
  var batteryPercent = bytes[8];
  var readingCount = bytes[9];

  return {
    data: {
      sensor_type: sensorType,
      distance_cm: distanceMm > 0 ? distanceMm / 10.0 : null,
      signal_strength: signalStrength,
      sensor_temp_c: sensorTemp,
      ambient_temp_c: ambientTemp !== -1 ? ambientTemp : null,
      humidity_percent: humidity > 0 ? humidity : null,
      battery_percent: batteryPercent,
      reading_count: readingCount,
      error: sensorType === 0xFF
    }
  };
}
```
