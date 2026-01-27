# TF-Nova LiDAR Sensor - Data Output Reference

## Overview

The Benewake TF-Nova is a compact LiDAR (Light Detection and Ranging) sensor that provides distance measurements using laser time-of-flight technology.

## Sensor Specifications

| Parameter | Value |
|-----------|-------|
| **Range** | 0.1m to 12m (10cm to 1200cm) |
| **Accuracy** | ±1cm typical |
| **Interface** | UART (default) or I2C |
| **Default Baud Rate** | 115200 |
| **Frame Rate** | Configurable (1-500 Hz) |
| **Operating Voltage** | 5V |
| **Current Consumption** | ~140mA peak |

## UART Frame Format

The TF-Nova outputs data frames continuously at the configured frame rate.

**Frame Structure (9 bytes):**
```
[0x59][0x59][Dist_L][Dist_H][Strength_L][Strength_H][Temp_L][Temp_H][Checksum]
```

| Byte | Description |
|------|-------------|
| 0-1 | Frame header (always 0x59, 0x59) |
| 2-3 | Distance (little-endian, in cm) |
| 4-5 | Signal strength (little-endian) |
| 6-7 | Temperature raw value (little-endian) |
| 8 | Checksum (sum of bytes 0-7, lower 8 bits) |

## Data Outputs

### 1. Distance (Primary Measurement)

| Field | Type | Unit | Range |
|-------|------|------|-------|
| distance_cm | int16_t | centimeters | 10 - 1200 |

**Notes:**
- Returns actual distance to target in centimeters
- Values outside 10-1200 cm range indicate invalid readings
- Distance of 0 or negative values indicate error/no target

**Usefulness for River Monitoring:** **HIGH** - Primary measurement for water level detection.

### 2. Signal Strength

| Field | Type | Unit | Range |
|-------|------|------|-------|
| signal_strength | int16_t | arbitrary | 0 - 65535 |

**Notes:**
- Higher values indicate stronger reflected signal
- Affected by:
  - Target surface reflectivity
  - Distance to target
  - Ambient light conditions
  - Surface angle

**Signal Strength Guidelines:**
| Value | Quality | Interpretation |
|-------|---------|----------------|
| < 100 | Poor | Measurement may be unreliable |
| 100-1000 | Fair | Acceptable for most measurements |
| > 1000 | Good | High confidence reading |

**Usefulness for River Monitoring:** **MEDIUM-HIGH**
- Can indicate water surface conditions (calm vs turbulent)
- Low strength may indicate measurement issues
- Can be used to filter out unreliable readings
- **Currently transmitted** in LoRa payload as `tfNovaStrength`

### 3. Internal Temperature

| Field | Type | Unit | Formula |
|-------|------|------|---------|
| temperature | int16_t | Celsius | (rawTemp / 8) - 256 |

**Notes:**
- This is the **internal chip temperature**, NOT ambient/environmental temperature
- Measures the temperature of the sensor's internal circuitry
- Raw value from sensor is converted using: `(rawValue / 8) - 256`

**Temperature Value Interpretation:**
| Range | Status | Action |
|-------|--------|--------|
| -20 to +40C | Normal | Normal operation |
| +40 to +60C | Warm | Monitor, may affect accuracy |
| > +60C | Hot | Consider cooling, accuracy degraded |
| < -20C | Cold | Warm up sensor, accuracy may be affected |

**Usefulness for River Monitoring:** **LOW-MEDIUM**
- **NOT suitable** for measuring ambient/water temperature (use DHT11 for that)
- Useful for:
  - Diagnostic purposes (detecting overheating)
  - Quality assurance (extreme temps affect accuracy)
  - Sensor health monitoring
  - Adjusting for temperature-induced drift (advanced)

## Current Implementation Status

### Data Being Used

| Data | Used | In Payload | Notes |
|------|------|------------|-------|
| Distance | Yes | `tfNovaDistMm` | Converted to mm for precision |
| Signal Strength | Yes | `tfNovaStrength` | For quality assessment |
| Temperature | Partial | No | Read but not transmitted |

### Available Methods in Library

```cpp
// Primary reading method
SensorReading read();          // Returns all data

// Individual accessors
int16_t readDistance();        // Distance in cm (-1 on error)
int16_t getSignalStrength();   // From last reading
int16_t getTemperature();      // Internal temp in Celsius

// Configuration
bool setFrameRate(uint16_t hz);  // Set output rate (1-500 Hz)
bool saveConfig();               // Save settings to sensor
bool factoryReset();             // Reset to defaults
```

## Recommendations

### Should We Transmit Internal Temperature?

**Not Recommended** for the following reasons:

1. **It's not ambient temperature** - The DHT11 already provides ambient temperature, which is more useful for river monitoring
2. **Limited utility** - Internal chip temp is mainly diagnostic
3. **Payload size** - Currently at 13 bytes, adding temp would increase size
4. **Power consumption** - More data = more transmission time

### Alternative: Use for Local Diagnostics

Instead of transmitting, use internal temperature for:
```cpp
// Example: Check sensor health before reading
int16_t chipTemp = tfNova.getTemperature();
if (chipTemp > 60) {
    Serial.println("Warning: TF-Nova running hot!");
    // Maybe delay reading or flag in status
}
```

### Signal Strength Best Practices

Currently transmitted - use it for:

1. **Quality filtering** on the server side:
   ```javascript
   // In TTN decoder or backend
   if (tfnova_signal < 100) {
       data.tfnova_quality = "low";
       // Maybe flag reading as uncertain
   }
   ```

2. **Surface condition detection**:
   - Consistent high strength = calm water
   - Variable strength = turbulent/wavy surface

## Comparison with DHT11 Temperature

| Sensor | Measures | Range | Accuracy | Use Case |
|--------|----------|-------|----------|----------|
| TF-Nova (internal) | Chip temperature | -20 to +70C | ~1C | Diagnostics |
| DHT11 | Ambient air | 0 to 50C | ±2C | Weather/environment |

**Conclusion:** Keep using DHT11 for environmental temperature. TF-Nova internal temp is only useful for sensor diagnostics.

## Wire Color Reference

| Wire Color | Function | Connect To |
|------------|----------|------------|
| Red | VCC (5V) | 5V power supply |
| Black | GND | Ground |
| Green | TX (sensor output) | ESP32 RX (GPIO 13) |
| White | RX (sensor input) | ESP32 TX (GPIO 14) |
