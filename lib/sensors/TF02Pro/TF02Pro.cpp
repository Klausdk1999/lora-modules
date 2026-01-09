/**
 * TF02Pro.cpp - Benewake TF02-Pro LiDAR Sensor Implementation
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include "TF02Pro.h"

bool TF02Pro::begin() {
    // Initialize serial with custom pins if specified
    #if defined(ESP32)
        if (_rxPin >= 0 && _txPin >= 0) {
            _serial->begin(_baud, SERIAL_8N1, _rxPin, _txPin);
        } else {
            _serial->begin(_baud);
        }
    #else
        _serial->begin(_baud);
    #endif
    
    delay(100);
    clearBuffer();
    
    // Try to read a frame to verify connection
    if (isConnected()) {
        _initialized = true;
        return true;
    }
    
    _initialized = false;
    return false;
}

bool TF02Pro::isConnected() {
    uint8_t buffer[TF02_FRAME_SIZE];
    uint32_t startTime = millis();
    
    // Wait up to 200ms for a valid frame
    while (millis() - startTime < 200) {
        if (readFrame(buffer)) {
            return true;
        }
        delay(10);
    }
    
    return false;
}

SensorReading TF02Pro::read() {
    SensorReading reading;
    reading.sensor_type = SensorType::TF02_PRO;
    reading.timestamp = millis();
    
    if (!_initialized) {
        reading.valid = false;
        return reading;
    }
    
    uint8_t buffer[TF02_FRAME_SIZE];
    
    // Try to read a frame
    if (!readFrame(buffer)) {
        reading.valid = false;
        return reading;
    }
    
    // Parse the frame
    if (!parseFrame(buffer)) {
        reading.valid = false;
        return reading;
    }
    
    // Check for valid distance
    if (_lastDist <= 0 || _lastDist > 2200) {
        reading.valid = false;
        return reading;
    }
    
    reading.valid = true;
    reading.distance_cm = (float)_lastDist;
    reading.distance_m = reading.distance_cm / 100.0f;
    reading.signal_strength = _lastStrength;
    reading.temperature = _lastTemp;
    
    return reading;
}

int16_t TF02Pro::readDistance() {
    SensorReading reading = read();
    if (reading.valid) {
        return (int16_t)reading.distance_cm;
    }
    return -1;
}

bool TF02Pro::readFrame(uint8_t* buffer) {
    uint32_t startTime = millis();
    int state = 0;
    int index = 0;
    
    while (millis() - startTime < 100) {
        if (_serial->available()) {
            uint8_t byte = _serial->read();
            
            switch (state) {
                case 0:  // Looking for first header byte
                    if (byte == TF02_FRAME_HEADER) {
                        buffer[0] = byte;
                        state = 1;
                    }
                    break;
                    
                case 1:  // Looking for second header byte
                    if (byte == TF02_FRAME_HEADER) {
                        buffer[1] = byte;
                        index = 2;
                        state = 2;
                    } else {
                        state = 0;
                    }
                    break;
                    
                case 2:  // Reading remaining bytes
                    buffer[index++] = byte;
                    if (index >= TF02_FRAME_SIZE) {
                        return true;
                    }
                    break;
            }
        }
    }
    
    return false;
}

bool TF02Pro::parseFrame(uint8_t* buffer) {
    // Verify header
    if (buffer[0] != TF02_FRAME_HEADER || buffer[1] != TF02_FRAME_HEADER) {
        return false;
    }
    
    // Calculate checksum (sum of bytes 0-7, lower 8 bits)
    uint8_t checksum = 0;
    for (int i = 0; i < 8; i++) {
        checksum += buffer[i];
    }
    
    if (checksum != buffer[8]) {
        return false;
    }
    
    // Parse data (little-endian)
    _lastDist = buffer[2] | (buffer[3] << 8);
    _lastStrength = buffer[4] | (buffer[5] << 8);
    _lastTemp = buffer[6] | (buffer[7] << 8);
    
    // Convert temperature (raw value is in 0.01°C, offset by 2731)
    // Formula: Temp_C = (raw - 2731) / 10
    _lastTemp = (_lastTemp - 2731) / 10;
    
    return true;
}

bool TF02Pro::setFrameRate(uint16_t frameRate) {
    // Command format: 5A 06 03 [rate_lo] [rate_hi] [checksum]
    uint8_t cmd[6] = {0x5A, 0x06, 0x03, 0x00, 0x00, 0x00};
    cmd[3] = frameRate & 0xFF;
    cmd[4] = (frameRate >> 8) & 0xFF;
    cmd[5] = cmd[0] + cmd[1] + cmd[2] + cmd[3] + cmd[4];
    
    return sendCommand(cmd, 6);
}

bool TF02Pro::factoryReset() {
    // Command: 5A 04 10 6E
    uint8_t cmd[4] = {0x5A, 0x04, 0x10, 0x6E};
    return sendCommand(cmd, 4);
}

void TF02Pro::clearBuffer() {
    while (_serial->available()) {
        _serial->read();
    }
}

bool TF02Pro::sendCommand(uint8_t* cmd, uint8_t len) {
    _serial->write(cmd, len);
    _serial->flush();
    delay(10);
    return true;
}






