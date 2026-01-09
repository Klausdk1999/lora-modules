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

## Adding a New Sensor

1. Create a new folder with the sensor name
2. Inherit from `SensorBase` class
3. Implement `begin()` and `read()` methods
4. Return a `SensorReading` struct with the measurement data






