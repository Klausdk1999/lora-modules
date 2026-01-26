/**
 * TFNova.cpp - Benewake TF-Nova LiDAR Sensor Driver Implementation
 *
 * UART Protocol:
 * - Default baud rate: 115200
 * - Frame format: [0x59][0x59][Dist_L][Dist_H][Strength_L][Strength_H][Temp_L][Temp_H][Checksum]
 * - Checksum: sum of bytes 0-7, lower 8 bits
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include "TFNova.h"

// ============================================================================
// Initialize the sensor
// ============================================================================
bool TFNova::begin() {
    // Initialize serial port with specified pins
    if (_rxPin >= 0 && _txPin >= 0) {
        _serial->begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    } else {
        _serial->begin(_baud);
    }

    // Wait for serial to initialize
    delay(100);

    // Clear any garbage in buffer
    clearBuffer();

    // Try to read a valid frame to confirm sensor is connected
    // Wait for sensor to start outputting data
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) {  // 2 second timeout
        SensorReading reading = read();
        if (reading.valid) {
            Serial.print(F("  TF-Nova detected: "));
            Serial.print(reading.distance_cm);
            Serial.print(F(" cm, strength: "));
            Serial.println(reading.signal_strength);
            return true;
        }
        delay(50);
    }

    Serial.println(F("  TF-Nova not detected (timeout waiting for data)"));
    return false;
}

// ============================================================================
// Read sensor data
// ============================================================================
SensorReading TFNova::read() {
    SensorReading result;
    result.valid = false;
    result.distance_cm = 0;
    result.signal_strength = 0;
    result.temperature = 0;

    uint8_t buffer[TFNOVA_FRAME_SIZE];

    if (readFrame(buffer)) {
        if (parseFrame(buffer)) {
            result.valid = true;
            result.distance_cm = _lastDist;
            result.signal_strength = _lastStrength;
            result.temperature = _lastTemp;
        }
    }

    return result;
}

// ============================================================================
// Read only distance
// ============================================================================
int16_t TFNova::readDistance() {
    SensorReading reading = read();
    if (reading.valid) {
        return reading.distance_cm;
    }
    return -1;
}

// ============================================================================
// Check if sensor is connected
// ============================================================================
bool TFNova::isConnected() {
    SensorReading reading = read();
    return reading.valid;
}

// ============================================================================
// Read a complete frame from serial
// ============================================================================
bool TFNova::readFrame(uint8_t* buffer) {
    unsigned long startTime = millis();

    // Wait for first header byte
    while (_serial->available() < TFNOVA_FRAME_SIZE) {
        if (millis() - startTime > TFNOVA_READ_TIMEOUT_MS) {
            return false;  // Timeout
        }
        delay(1);
    }

    // Look for frame header (two 0x59 bytes)
    while (_serial->available() >= TFNOVA_FRAME_SIZE) {
        uint8_t byte1 = _serial->peek();

        if (byte1 != TFNOVA_FRAME_HEADER) {
            _serial->read();  // Discard and continue looking
            continue;
        }

        // Found potential header, read first byte
        buffer[0] = _serial->read();

        // Check second header byte
        if (_serial->available() < 1) return false;

        uint8_t byte2 = _serial->peek();
        if (byte2 != TFNOVA_FRAME_HEADER) {
            // Not a valid frame, continue looking
            continue;
        }

        buffer[1] = _serial->read();

        // Read rest of frame
        if (_serial->available() < TFNOVA_FRAME_SIZE - 2) {
            // Wait for remaining bytes
            unsigned long waitStart = millis();
            while (_serial->available() < TFNOVA_FRAME_SIZE - 2) {
                if (millis() - waitStart > TFNOVA_READ_TIMEOUT_MS) {
                    return false;
                }
                delay(1);
            }
        }

        for (int i = 2; i < TFNOVA_FRAME_SIZE; i++) {
            buffer[i] = _serial->read();
        }

        return true;
    }

    return false;
}

// ============================================================================
// Parse frame and extract data
// ============================================================================
bool TFNova::parseFrame(uint8_t* buffer) {
    // Verify headers
    if (buffer[0] != TFNOVA_FRAME_HEADER || buffer[1] != TFNOVA_FRAME_HEADER) {
        return false;
    }

    // Calculate checksum (sum of bytes 0-7, lower 8 bits)
    uint8_t checksum = 0;
    for (int i = 0; i < TFNOVA_FRAME_SIZE - 1; i++) {
        checksum += buffer[i];
    }

    if (checksum != buffer[TFNOVA_FRAME_SIZE - 1]) {
        return false;  // Checksum mismatch
    }

    // Extract data (little-endian)
    _lastDist = buffer[2] | (buffer[3] << 8);           // Distance in cm
    _lastStrength = buffer[4] | (buffer[5] << 8);       // Signal strength
    int16_t rawTemp = buffer[6] | (buffer[7] << 8);     // Raw temperature
    _lastTemp = (rawTemp / 8) - 256;                    // Convert to Celsius

    return true;
}

// ============================================================================
// Clear serial buffer
// ============================================================================
void TFNova::clearBuffer() {
    while (_serial->available()) {
        _serial->read();
    }
}

// ============================================================================
// Set frame rate
// ============================================================================
bool TFNova::setFrameRate(uint16_t frameRate) {
    // Command format: 5A 06 03 [rate_L] [rate_H] [checksum]
    uint8_t cmd[6];
    cmd[0] = 0x5A;
    cmd[1] = 0x06;
    cmd[2] = 0x03;
    cmd[3] = frameRate & 0xFF;
    cmd[4] = (frameRate >> 8) & 0xFF;
    cmd[5] = (cmd[0] + cmd[1] + cmd[2] + cmd[3] + cmd[4]) & 0xFF;

    return sendCommand(cmd, 6);
}

// ============================================================================
// Save configuration
// ============================================================================
bool TFNova::saveConfig() {
    // Command: 5A 04 11 [checksum]
    uint8_t cmd[4] = {0x5A, 0x04, 0x11, 0x6F};
    return sendCommand(cmd, 4);
}

// ============================================================================
// Factory reset
// ============================================================================
bool TFNova::factoryReset() {
    // Command: 5A 04 10 [checksum]
    uint8_t cmd[4] = {0x5A, 0x04, 0x10, 0x6E};
    return sendCommand(cmd, 4);
}

// ============================================================================
// Send command to sensor
// ============================================================================
bool TFNova::sendCommand(uint8_t* cmd, uint8_t len) {
    _serial->write(cmd, len);
    _serial->flush();
    delay(10);  // Give sensor time to process

    // Clear any response
    clearBuffer();

    return true;
}
