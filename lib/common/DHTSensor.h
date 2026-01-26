/**
 * DHTSensor.h - DHT11/DHT22 Temperature Sensor Wrapper
 *
 * Implements the ITemperatureSensor interface for DHT series sensors.
 * Uses the Adafruit DHT library.
 *
 * Wiring:
 * - VCC: 3.3V or 5V
 * - GND: Ground
 * - DATA: Any GPIO pin (with 10K pullup recommended)
 *
 * Note: DHT sensors need 2 seconds between readings for accuracy.
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef DHT_SENSOR_WRAPPER_H
#define DHT_SENSOR_WRAPPER_H

#include "TemperatureSensor.h"
#include <DHT.h>

class DHTSensor : public ITemperatureSensor {
public:
    /**
     * Constructor
     * @param pin GPIO pin connected to DHT data line
     * @param type DHT type: DHT11 or DHT22
     */
    DHTSensor(uint8_t pin, uint8_t type = DHT11)
        : _dht(pin, type),
          _pin(pin),
          _type(type),
          _initialized(false),
          _available(false) {}

    /**
     * Initialize the DHT sensor
     */
    bool begin() override {
        Serial.print(F("Initializing DHT sensor on GPIO "));
        Serial.println(_pin);

        _dht.begin();

        // Wait for sensor to stabilize
        delay(2000);

        // Test reading
        float temp = _dht.readTemperature();
        if (!isnan(temp) && TemperatureSensorManager::isValidTemperature(temp)) {
            _initialized = true;
            _available = true;
            Serial.print(F("  DHT sensor initialized: "));
            Serial.print(temp);
            Serial.println(F(" C"));
            return true;
        }

        Serial.println(F("  DHT sensor NOT detected or unavailable"));
        Serial.println(F("  Check wiring: VCC->3.3V/5V, GND->GND, DATA->GPIO"));
        _initialized = true;  // Mark as initialized even if unavailable
        _available = false;
        return false;
    }

    /**
     * Read temperature and humidity
     */
    TemperatureReading read() override {
        TemperatureReading reading;
        reading.timestamp = millis();
        reading.source = (_type == DHT11) ? TEMP_SENSOR_DHT11 : TEMP_SENSOR_DHT22;

        if (!_available) {
            reading.temperature = DEFAULT_TEMPERATURE_C;
            reading.humidity = 0.0f;
            reading.valid = false;
            return reading;
        }

        // Read temperature and humidity
        reading.temperature = _dht.readTemperature();
        reading.humidity = _dht.readHumidity();

        // Validate readings
        if (isnan(reading.temperature) || isnan(reading.humidity)) {
            reading.valid = false;
            reading.temperature = DEFAULT_TEMPERATURE_C;
            reading.humidity = 0.0f;
            return reading;
        }

        // Check temperature range
        if (!TemperatureSensorManager::isValidTemperature(reading.temperature)) {
            Serial.print(F("DHT reading out of range: "));
            Serial.print(reading.temperature);
            Serial.println(F(" C"));
            reading.valid = false;
            return reading;
        }

        reading.valid = true;
        return reading;
    }

    TemperatureSensorType getType() const override {
        return (_type == DHT11) ? TEMP_SENSOR_DHT11 : TEMP_SENSOR_DHT22;
    }

    bool isAvailable() const override {
        return _available;
    }

    /**
     * Get the GPIO pin number
     */
    uint8_t getPin() const {
        return _pin;
    }

private:
    DHT _dht;
    uint8_t _pin;
    uint8_t _type;
    bool _initialized;
    bool _available;
};

#endif // DHT_SENSOR_WRAPPER_H
