/**
 * TemperatureSensor.h - Unified Temperature Sensor Interface
 *
 * Provides a common interface for reading temperature from various sensors
 * (DHT11, DHT22, or internal sensor readings from LiDAR).
 *
 * The temperature reading is critical for:
 * - Speed of sound compensation in ultrasonic sensors
 * - Thermal drift compensation in LiDAR sensors
 *
 * Based on findings from:
 * - Mohammed et al. (2019): Temperature compensation critical for sub-cm accuracy
 * - Tawalbeh et al. (2023): Diurnal swings of 20C can cause several cm error
 * - Panagopoulos et al. (2021): 1-2cm drift per 10C without compensation
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include <Arduino.h>

// Default temperature when sensor is unavailable
#define DEFAULT_TEMPERATURE_C   25.0f

// Valid temperature range for outdoor deployment
#define MIN_VALID_TEMP_C        -20.0f
#define MAX_VALID_TEMP_C        60.0f

// Invalid temperature sentinel value
#define INVALID_TEMP_VALUE      -128

/**
 * Temperature Sensor Type Enumeration
 */
enum TemperatureSensorType {
    TEMP_SENSOR_NONE = 0,
    TEMP_SENSOR_DHT11,
    TEMP_SENSOR_DHT22,
    TEMP_SENSOR_INTERNAL,  // LiDAR internal temperature
    TEMP_SENSOR_DS18B20
};

/**
 * Temperature Reading Structure
 */
struct TemperatureReading {
    float temperature;              // Temperature in Celsius
    float humidity;                 // Humidity in % (if available)
    bool valid;                     // True if reading is valid
    TemperatureSensorType source;   // Source of the reading
    uint32_t timestamp;             // Millis when reading was taken
};

/**
 * Abstract Temperature Sensor Interface
 */
class ITemperatureSensor {
public:
    virtual ~ITemperatureSensor() {}

    /**
     * Initialize the temperature sensor
     * @return true if initialization successful
     */
    virtual bool begin() = 0;

    /**
     * Read temperature and humidity
     * @return TemperatureReading structure with data
     */
    virtual TemperatureReading read() = 0;

    /**
     * Get the sensor type
     */
    virtual TemperatureSensorType getType() const = 0;

    /**
     * Check if sensor is available
     */
    virtual bool isAvailable() const = 0;
};

/**
 * Default/Fallback Temperature Provider
 * Returns a constant default temperature when no sensor is available
 */
class DefaultTemperatureSensor : public ITemperatureSensor {
public:
    DefaultTemperatureSensor(float defaultTemp = DEFAULT_TEMPERATURE_C)
        : _defaultTemp(defaultTemp), _initialized(false) {}

    bool begin() override {
        _initialized = true;
        return true;
    }

    TemperatureReading read() override {
        TemperatureReading reading;
        reading.temperature = _defaultTemp;
        reading.humidity = 0.0f;
        reading.valid = true;
        reading.source = TEMP_SENSOR_NONE;
        reading.timestamp = millis();
        return reading;
    }

    TemperatureSensorType getType() const override {
        return TEMP_SENSOR_NONE;
    }

    bool isAvailable() const override {
        return _initialized;
    }

    void setDefaultTemperature(float temp) {
        _defaultTemp = temp;
    }

private:
    float _defaultTemp;
    bool _initialized;
};

/**
 * Temperature Sensor Manager
 *
 * Manages temperature reading with automatic fallback to default
 * if the primary sensor fails.
 */
class TemperatureSensorManager {
public:
    TemperatureSensorManager()
        : _primarySensor(nullptr),
          _defaultSensor(DEFAULT_TEMPERATURE_C),
          _lastValidReading(),
          _consecutiveFailures(0),
          _maxConsecutiveFailures(3) {
        _lastValidReading.temperature = DEFAULT_TEMPERATURE_C;
        _lastValidReading.valid = true;
        _lastValidReading.source = TEMP_SENSOR_NONE;
    }

    /**
     * Set the primary temperature sensor
     */
    void setPrimarySensor(ITemperatureSensor* sensor) {
        _primarySensor = sensor;
    }

    /**
     * Set the default temperature for fallback
     */
    void setDefaultTemperature(float temp) {
        _defaultSensor.setDefaultTemperature(temp);
    }

    /**
     * Initialize all sensors
     */
    bool begin() {
        _defaultSensor.begin();

        if (_primarySensor != nullptr) {
            return _primarySensor->begin();
        }
        return true;
    }

    /**
     * Read temperature with automatic fallback
     *
     * If the primary sensor fails, uses the last valid reading or default.
     * This ensures temperature compensation always has a reasonable value.
     */
    TemperatureReading read() {
        // Try primary sensor first
        if (_primarySensor != nullptr && _primarySensor->isAvailable()) {
            TemperatureReading reading = _primarySensor->read();

            if (reading.valid && isValidTemperature(reading.temperature)) {
                _consecutiveFailures = 0;
                _lastValidReading = reading;
                return reading;
            } else {
                _consecutiveFailures++;
                Serial.print(F("Temperature reading failed ("));
                Serial.print(_consecutiveFailures);
                Serial.println(F(" consecutive)"));
            }
        }

        // Primary sensor unavailable or failed - use fallback
        if (_consecutiveFailures < _maxConsecutiveFailures &&
            _lastValidReading.valid) {
            // Use last valid reading if recent failures
            Serial.print(F("Using last valid temperature: "));
            Serial.println(_lastValidReading.temperature);
            return _lastValidReading;
        }

        // Fall back to default
        Serial.print(F("Using default temperature: "));
        Serial.println(DEFAULT_TEMPERATURE_C);
        return _defaultSensor.read();
    }

    /**
     * Get the last valid temperature reading
     */
    float getTemperature() {
        TemperatureReading reading = read();
        return reading.temperature;
    }

    /**
     * Check if temperature is within valid range
     */
    static bool isValidTemperature(float temp) {
        return (temp >= MIN_VALID_TEMP_C && temp <= MAX_VALID_TEMP_C);
    }

    /**
     * Check if primary sensor is available
     */
    bool isPrimaryAvailable() const {
        return (_primarySensor != nullptr && _primarySensor->isAvailable());
    }

private:
    ITemperatureSensor* _primarySensor;
    DefaultTemperatureSensor _defaultSensor;
    TemperatureReading _lastValidReading;
    uint8_t _consecutiveFailures;
    uint8_t _maxConsecutiveFailures;
};

#endif // TEMPERATURE_SENSOR_H
