/**
 * TFLuna.cpp - Benewake TF-Luna LiDAR Sensor Implementation
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include "TFLuna.h"

bool TFLuna::begin() {
    _wire->begin();
    delay(100); // Give sensor time to initialize
    
    // Try to read distance to verify sensor is connected
    if (isConnected()) {
        _initialized = true;
        return true;
    }
    
    _initialized = false;
    return false;
}

bool TFLuna::isConnected() {
    _wire->beginTransmission(_i2cAddr);
    uint8_t error = _wire->endTransmission();
    return (error == 0);
}

SensorReading TFLuna::read() {
    SensorReading reading;
    reading.sensor_type = SensorType::TF_LUNA;
    reading.timestamp = millis();
    
    if (!_initialized) {
        reading.valid = false;
        return reading;
    }
    
    // Read 6 bytes starting from distance register
    uint8_t buffer[6];
    if (!readRegisters(TFL_DIST_LO, buffer, 6)) {
        reading.valid = false;
        return reading;
    }
    
    // Parse data (little-endian format)
    _lastDist = buffer[0] | (buffer[1] << 8);
    _lastFlux = buffer[2] | (buffer[3] << 8);
    _lastTemp = buffer[4] | (buffer[5] << 8);
    
    // Validate reading
    // TF-Luna returns 0 or negative values when out of range
    if (_lastDist <= 0 || _lastFlux < 100) {
        reading.valid = false;
        return reading;
    }
    
    reading.valid = true;
    reading.distance_cm = (float)_lastDist;
    reading.distance_m = reading.distance_cm / 100.0f;
    reading.signal_strength = _lastFlux;
    reading.temperature = _lastTemp / 100;  // Convert to Celsius
    
    return reading;
}

int16_t TFLuna::readDistance() {
    uint8_t buffer[2];
    if (!readRegisters(TFL_DIST_LO, buffer, 2)) {
        return -1;
    }
    _lastDist = buffer[0] | (buffer[1] << 8);
    return _lastDist;
}

int16_t TFLuna::readFlux() {
    uint8_t buffer[2];
    if (!readRegisters(TFL_FLUX_LO, buffer, 2)) {
        return -1;
    }
    _lastFlux = buffer[0] | (buffer[1] << 8);
    return _lastFlux;
}

int16_t TFLuna::readTemperature() {
    uint8_t buffer[2];
    if (!readRegisters(TFL_TEMP_LO, buffer, 2)) {
        return -1;
    }
    _lastTemp = buffer[0] | (buffer[1] << 8);
    return _lastTemp / 100;  // Return in Celsius
}

bool TFLuna::setI2CAddress(uint8_t newAddr) {
    // Address must be in valid I2C range
    if (newAddr < 0x08 || newAddr > 0x77) {
        return false;
    }
    
    // Write new address to sensor
    // This is a permanent change that survives power cycles
    if (!writeRegister(0x22, newAddr)) {
        return false;
    }
    
    // Save settings
    if (!writeRegister(0x20, 0x01)) {
        return false;
    }
    
    _i2cAddr = newAddr;
    return true;
}

bool TFLuna::readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t count) {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(startReg);
    _lastError = _wire->endTransmission(false);
    
    if (_lastError != 0) {
        return false;
    }
    
    uint8_t received = _wire->requestFrom(_i2cAddr, count);
    if (received != count) {
        _lastError = 4; // Timeout
        return false;
    }
    
    for (uint8_t i = 0; i < count; i++) {
        buffer[i] = _wire->read();
    }
    
    return true;
}

bool TFLuna::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg);
    _wire->write(value);
    _lastError = _wire->endTransmission();
    return (_lastError == 0);
}






