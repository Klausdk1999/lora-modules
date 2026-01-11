# Sensor Libraries for River Level Monitoring

This folder contains reusable sensor driver libraries for the river level monitoring project.
Each sensor has its own folder with a header file (.h) and implementation file (.cpp).

## Supported Sensors

### LiDAR Sensors (Benewake)
| Sensor | Range | Interface | File |
|--------|-------|-----------|------|
| TF-Luna | 0.2m - 8m | I2C/UART | `TFLuna/` |
| TF-Mini Plus | 0.1m - 12m | I2C/UART | `TFMiniPlus/` |
| TF02 Pro | 0.1m - 22m | UART | `TF02Pro/` |

### Ultrasonic Sensors
| Sensor | Range | Interface | File |
|--------|-------|-----------|------|
| HC-SR04 | 2cm - 400cm | Digital GPIO | `HCSR04/` |
| AJ-SR04M | 20cm - 800cm | Digital GPIO | `AJSR04M/` |
| JSN-SR04T | 25cm - 450cm | Digital GPIO | `JSNSR04T/` |

## Usage Example

```cpp
#include "sensors/TFLuna/TFLuna.h"
#include "sensors/HCSR04/HCSR04.h"

// Create sensor instances
TFLuna lidar;
HCSR04 ultrasonic(TRIG_PIN, ECHO_PIN);

void setup() {
    lidar.begin();
    ultrasonic.begin();
}

void loop() {
    SensorReading lidarReading = lidar.read();
    SensorReading ultrasonicReading = ultrasonic.read();
    
    if (lidarReading.valid) {
        Serial.println(lidarReading.distance_cm);
    }
}
```

## Folder Structure

```
sensors/
├── README.md
├── SensorBase.h          # Base class for all sensors
├── TFLuna/
│   ├── TFLuna.h
│   └── TFLuna.cpp
├── TF02Pro/
│   ├── TF02Pro.h
│   └── TF02Pro.cpp
├── HCSR04/
│   ├── HCSR04.h
│   └── HCSR04.cpp
├── AJSR04M/
│   ├── AJSR04M.h
│   └── AJSR04M.cpp
└── JSNSR04T/
    ├── JSNSR04T.h
    └── JSNSR04T.cpp
```

## Temperature Compensation

### Ultrasonic Sensors (HC-SR04, JSN-SR04T)

Ultrasonic sensors require temperature compensation for accurate measurements. The speed of sound in air varies with temperature, and this must be accounted for to achieve sub-centimeter accuracy (Mohammed et al. 2019, Tawalbeh et al. 2023).

**Speed-of-Sound Formula:**
```
v(T) = 331.3 + 0.606 * θ
```
Where:
- `v(T)` = speed of sound in m/s at temperature T
- `θ` = ambient temperature in Celsius

**Implementation:**
```cpp
#include "sensors/JSNSR04T/JSNSR04T.h"
#include "DHTesp.h"

DHTesp dhtSensor;
JSNSR04T ultrasonic(TRIG_PIN, ECHO_PIN);

void setup() {
    dhtSensor.setup(DHT_PIN, DHTesp::DHT11);
    ultrasonic.begin();
}

void loop() {
    // Get ambient temperature (Mohammed et al. 2019, Tawalbeh et al. 2023)
    TempAndHumidity tempHumid = dhtSensor.getTempAndHumidity();
    ultrasonic.setTemperature(tempHumid.temperature);
    
    // Read distance with temperature compensation
    float distance = ultrasonic.readDistanceCm();
}
```

**Key Findings:**
- **Mohammed et al. (2019)**: Temperature compensation reduces RMSE, achieving sub-centimeter accuracy with integrated temperature sensor (DHT11/DHT22)
- **Tawalbeh et al. (2023)**: Diurnal temperature swings of 20°C can introduce measurement errors of several centimeters without compensation
- **Temperature sensor placement**: Must be physically located near the ultrasonic transducer for accurate air column density measurement (Mohammed et al. 2019)

**Supported Temperature Sensors:**
- DHT11 (primary, ±2°C accuracy, sufficient for speed-of-sound compensation)
- DHT22 (optional, ±0.5°C accuracy)
- BMP280 (optional secondary sensor, more accurate but overkill for this application)

### LiDAR Sensors (TF02-Pro)

TF02-Pro LiDAR sensors have internal temperature sensors and automatically compensate for temperature variations. However, additional compensation may be applied in software:

**Temperature Compensation Formula:**
```cpp
// Based on Mohammed et al. (2019) and Tawalbeh et al. (2023)
// LiDAR sensor accuracy is strongly dependent on internal temperature
// Correction factor: approximately 0.1% per degree Celsius deviation from 25°C
float compensatedDistance = rawDistance * (1.0f + ((temperature - 25.0f) * 0.001f));
```

**Key Findings:**
- **Mohammed et al. (2019)**: LiDAR sensor accuracy strongly dependent on internal temperature
- **Tawalbeh et al. (2023)**: Temperature compensation critical for achieving sub-centimeter accuracy
- **Reference temperature**: 25°C (typical room temperature)

## Statistical Filtering

Raw sensor data in river environments is prone to noise from turbulence, debris, and environmental factors. Statistical filtering is required for reliable measurements (Kabi et al. 2023).

**Filtering Approach (Kabi et al. 2023):**
- **Median filtering**: More robust than mean for noisy data
- **Outlier removal**: Reject readings >20% deviation from median
- **Multiple readings**: Take 7 readings per measurement cycle (odd number recommended)

**Implementation:**
```cpp
// Collect multiple readings
float readings[NUM_READINGS_AVG];
uint8_t validReadings = 0;

for (int i = 0; i < NUM_READINGS_AVG; i++) {
    SensorReading reading = sensor.read();
    if (reading.valid && reading.distance_cm > 0) {
        readings[validReadings++] = reading.distance_cm;
    }
    delay(100);  // Delay between readings
}

// Calculate median for robust filtering
float median = calculateMedian(readings, validReadings);

// Filter outliers based on deviation from median
float filteredDistance = filterOutliers(readings, validReadings, median, OUTLIER_THRESHOLD_PCT);
```

## Academic References

### Temperature Compensation
- **Mohammed et al. (2019)**: "Highly Accurate Water Level Measurement System" - Temperature compensation critical for sub-centimeter accuracy
- **Tawalbeh et al. (2023)**: "Evaluation of Ultrasonic Sensors" - Diurnal temperature swings of 20°C cause several centimeters error without compensation

### Statistical Filtering
- **Kabi et al. (2023)**: Statistical filtering required for raw sensor data in river environments (noise from turbulence and debris)

### Sensor Validation
- **Mohammadreza MasoudiMoghaddam et al. (2024)**: JSN-SR04T validated for water level monitoring
- **Panagopoulos et al. (2021)**: Ultrasonic sensor validation, 1-2cm drift per 10°C without compensation
- **Santana et al. (2024)**: TF02-Pro LiDAR sensor (22m range) validated for larger rivers

## Adding a New Sensor

1. Create a new folder with the sensor name
2. Inherit from `SensorBase` class
3. Implement `begin()` and `read()` methods
4. Return a `SensorReading` struct with the measurement data
5. If applicable, implement temperature compensation (ultrasonic sensors) or use internal compensation (LiDAR sensors)
6. Consider implementing statistical filtering for noisy environments






