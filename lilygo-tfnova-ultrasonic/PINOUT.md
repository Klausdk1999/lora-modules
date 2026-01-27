# LilyGo T-Beam V1.2 - Multi-Sensor Node Pinout

## Overview
This node uses a LilyGo T-Beam V1.2 (AXP2101) with:
- TF-Nova LiDAR sensor (UART)
- AJ-SR04M Ultrasonic sensor (GPIO)
- DHT11 Temperature/Humidity sensor

## T-Beam V1.2 Available External Pins

```
Header Side 1:           Header Side 2:
  VP (GPIO 36)             TX (GPIO 1)
  VN (GPIO 39)             RX (GPIO 3)
  RST                      23
  15                       4
  35                       0
  32                       GND
  33                       3V3
  25                       GND
  14                       22
  13                       21
  2                        3.3V
  GND                      LORA2 (internal)
  5V                       LORA1 (internal)
```

**Note**: GPIO 36 (VP), 39 (VN), and 35 are input-only pins.

## Power Supply
- **5V External Battery** powers the TF-Nova and AJ-SR04M sensors
- Both sensors require 5V (they will not work reliably on 3.3V)
- DHT11 can work on 3.3V or 5V
- The T-Beam's onboard battery powers the ESP32 and LoRa module

## Wiring Diagram (ASCII)

```
                    +-------------------+
                    |   LilyGo T-Beam   |
                    |     V1.2          |
                    |    (AXP2101)      |
                    +-------------------+
                    |                   |
    5V Battery -----| 5V     GND |------+---- GND (common)
                    |                   |
                    | GPIO 13 (RX) |----+---- TF-Nova TX (green)
                    | GPIO 14 (TX) |----+---- TF-Nova RX (white)
                    |                   |
                    | GPIO 25 (TRIG)|---+---- AJ-SR04M TRIG
                    | GPIO 35 (ECHO)|---+---- AJ-SR04M ECHO (input-only)
                    |                   |
                    | GPIO 15 (DATA)|---+---- DHT11 DATA
                    |                   |
                    | GPIO 21 (SDA) |---+---- (I2C for PMU, internal)
                    | GPIO 22 (SCL) |---+---- (I2C for PMU, internal)
                    +-------------------+


    TF-Nova LiDAR              AJ-SR04M Ultrasonic         DHT11
    +-------------+            +------------------+        +---------+
    | VCC (red)   |---5V       | VCC              |---5V   | VCC     |---3.3V/5V
    | GND (black) |---GND      | GND              |---GND  | GND     |---GND
    | TX (green)  |---GPIO 13  | TRIG             |---GPIO 25
    | RX (white)  |---GPIO 14  | ECHO             |---GPIO 35| DATA    |---GPIO 15
    +-------------+            +------------------+        +---------+
```

## Pin Assignment Table

| Function          | GPIO | Direction | Connected To        | Notes                          |
|-------------------|------|-----------|---------------------|--------------------------------|
| **LoRa (SPI)**    |      |           |                     | Internal - DO NOT USE          |
| NSS               | 18   | Output    | LoRa SX1276         | Chip select                    |
| RST               | 23   | Output    | LoRa SX1276         | Reset                          |
| DIO0              | 26   | Input     | LoRa SX1276         | TX/RX complete interrupt       |
| DIO1              | 33   | Input     | LoRa SX1276         | RX timeout                     |
| DIO2              | 32   | Input     | LoRa SX1276         | FSK mode (unused)              |
| **PMU (I2C)**     |      |           |                     | Internal - AXP2101             |
| SDA               | 21   | Bidirect  | AXP2101             | I2C data                       |
| SCL               | 22   | Output    | AXP2101             | I2C clock                      |
| **TF-Nova (UART)**|      |           |                     | Serial2                        |
| RX                | 13   | Input     | TF-Nova TX          | Receives sensor data           |
| TX                | 14   | Output    | TF-Nova RX          | Sends commands                 |
| **AJ-SR04M**      |      |           |                     | GPIO trigger/echo              |
| TRIG              | 25   | Output    | AJ-SR04M TRIG       | Trigger pulse                  |
| ECHO              | 35   | Input     | AJ-SR04M ECHO       | Echo return (input-only pin)   |
| **DHT11**         |      |           |                     | Single-wire protocol           |
| DATA              | 15   | Bidirect  | DHT11 DATA          | Temperature/humidity           |

## Sensor Specifications

### TF-Nova LiDAR
- **Interface**: UART @ 115200 baud
- **Range**: 0.1m to 12m
- **Accuracy**: ±1cm typical
- **Power**: 5V, ~140mA peak
- **Frame format**: [0x59][0x59][Dist_L][Dist_H][Str_L][Str_H][Temp_L][Temp_H][Checksum]

### AJ-SR04M Ultrasonic
- **Interface**: GPIO (trigger/echo)
- **Range**: 20cm to 800cm
- **Accuracy**: ±0.5cm
- **Power**: 5V, ~30mA
- **Waterproof**: IP67 probe

### DHT11
- **Interface**: Single-wire digital
- **Temperature range**: 0-50°C, ±2°C accuracy
- **Humidity range**: 20-80%, ±5% accuracy
- **Power**: 3.3V-5V, ~2.5mA

## Power Considerations

1. **TF-Nova and AJ-SR04M require 5V** - Use external 5V battery/boost converter
2. **Total current draw** (peak):
   - TF-Nova: ~140mA
   - AJ-SR04M: ~30mA
   - DHT11: ~2.5mA
   - ESP32 active: ~180mA
   - **Total**: ~350mA peak
3. **Deep sleep current**: ~10µA (ESP32 only)

## Troubleshooting

### TF-Nova not responding
1. Check 5V power supply is delivering enough current (>200mA)
2. Verify TX/RX wiring (sensor TX -> ESP RX, sensor RX -> ESP TX)
3. Check baud rate is 115200
4. Try power cycling the sensor

### AJ-SR04M timeout
1. Ensure target is within range (20-800cm)
2. Check for obstructions in the acoustic path
3. Verify 5V power (3.3V will not work reliably)
4. Check TRIG (GPIO 25) / ECHO (GPIO 35) pin connections

### DHT11 NaN readings
1. Wait 2 seconds after power-on before reading
2. Add 10K pullup resistor on DATA line
3. Ensure minimum 1 second between readings
4. Verify VCC is connected (3.3V or 5V)

## LoRa Payload Format (16 bytes)

```
Byte  Field              Type     Description
0     sensorFlags        uint8    Bit flags (b0=TFNova OK, b1=US OK, b2=DHT OK, b7=error)
1-2   tfNovaDistMm       int16    TF-Nova distance in mm (-1 = error)
3-4   tfNovaStrength     int16    TF-Nova signal strength
5-6   ultrasonicDistMm   int16    Ultrasonic distance in mm (-1 = error)
7     temperature        int8     DHT11 temperature (Celsius)
8     humidity           uint8    DHT11 humidity (%)
9     batteryPercent     uint8    Battery level (0-100%)
10-11 batteryMv          uint16   Battery voltage in mV
12    readingCount       uint8    Number of valid sensor readings
```

## TTN Payload Decoder

```javascript
function decodeUplink(input) {
  var bytes = input.bytes;

  var sensorFlags = bytes[0];
  var tfNovaOk = (sensorFlags & 0x01) !== 0;
  var ultrasonicOk = (sensorFlags & 0x02) !== 0;
  var dhtOk = (sensorFlags & 0x04) !== 0;
  var hasError = (sensorFlags & 0x80) !== 0;

  var tfNovaDistMm = (bytes[2] << 8) | bytes[1];
  if (tfNovaDistMm > 32767) tfNovaDistMm -= 65536;

  var tfNovaStrength = (bytes[4] << 8) | bytes[3];
  if (tfNovaStrength > 32767) tfNovaStrength -= 65536;

  var ultrasonicDistMm = (bytes[6] << 8) | bytes[5];
  if (ultrasonicDistMm > 32767) ultrasonicDistMm -= 65536;

  var temperature = bytes[7];
  if (temperature > 127) temperature -= 256;

  var humidity = bytes[8];
  var batteryPercent = bytes[9];

  var batteryMv = (bytes[11] << 8) | bytes[10];

  var readingCount = bytes[12];

  return {
    data: {
      sensor_flags: sensorFlags,
      tfnova_ok: tfNovaOk,
      ultrasonic_ok: ultrasonicOk,
      dht_ok: dhtOk,
      has_error: hasError,
      tfnova_distance_cm: tfNovaDistMm > 0 ? tfNovaDistMm / 10.0 : null,
      tfnova_signal: tfNovaStrength,
      ultrasonic_distance_cm: ultrasonicDistMm > 0 ? ultrasonicDistMm / 10.0 : null,
      temperature_c: dhtOk ? temperature : null,
      humidity_percent: dhtOk ? humidity : null,
      battery_percent: batteryPercent,
      battery_mv: batteryMv,
      reading_count: readingCount
    }
  };
}
```
