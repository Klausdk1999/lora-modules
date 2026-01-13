/**
 * TF02Pro.cpp - Benewake TF02-Pro LiDAR Sensor Implementation
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include "TF02Pro.h"

bool TF02Pro::begin() {
    // Use Serial.print with explicit flush to avoid interference
    Serial.print(F("[TF02Pro] begin() - RX:"));
    Serial.print(_rxPin);
    Serial.print(F(" TX:"));
    Serial.println(_txPin);
    delay(50);  // Small delay to ensure Serial output is sent
    
    // Initialize serial with custom pins if specified
    #if defined(ESP32)
        if (_rxPin >= 0 && _txPin >= 0) {
            Serial.println(F("[TF02Pro] Init Serial2..."));
            delay(50);
            _serial->begin(_baud, SERIAL_8N1, _rxPin, _txPin);
            Serial.println(F("[TF02Pro] Serial2 OK"));
            delay(50);
        } else {
            Serial.println(F("[TF02Pro] Init Serial2 default..."));
            delay(50);
            _serial->begin(_baud);
            Serial.println(F("[TF02Pro] Serial2 OK"));
            delay(50);
        }
    #else
        _serial->begin(_baud);
    #endif
    
    // TF02-Pro needs time to start sending data after power-on
    Serial.println(F("[TF02Pro] Wait 500ms..."));
    delay(500);
    
    Serial.println(F("[TF02Pro] Clear buffer..."));
    clearBuffer();
    delay(50);
    
    // Try to read a frame to verify connection
    // TF02-Pro sends data continuously, so we should receive a frame if connected
    Serial.println(F("[TF02Pro] Check connection..."));
    delay(50);
    
    if (isConnected()) {
        Serial.println(F("[TF02Pro] CONNECTED!"));
        delay(50);
        _initialized = true;
        return true;
    }
    
    Serial.println(F("[TF02Pro] NOT CONNECTED!"));
    delay(50);
    _initialized = false;
    return false;
}

bool TF02Pro::isConnected() {
    uint8_t buffer[TF02_FRAME_SIZE];
    uint32_t startTime = millis();
    uint32_t bytesReceived = 0;
    uint32_t lastBytesCount = 0;
    uint32_t lastDebugTime = 0;
    
    Serial.println(F("[isConnected] Wait 2000ms..."));
    delay(50);
    
    // Wait up to 2000ms for a valid frame (TF02-Pro needs time to start sending data)
    while (millis() - startTime < 2000) {
        // Check if we're receiving any data at all
        uint32_t available = _serial->available();
        if (available > 0) {
            bytesReceived += available;
            
            // Print debug every 300ms or when bytes count changes significantly
            if ((millis() - lastDebugTime > 300) || (bytesReceived - lastBytesCount > 10)) {
                Serial.print(F("[isConnected] RX:"));
                Serial.print(bytesReceived);
                Serial.print(F(" avail:"));
                Serial.println(available);
                delay(50);
                lastDebugTime = millis();
                lastBytesCount = bytesReceived;
            }
            
            if (readFrame(buffer)) {
                uint16_t dist = buffer[2] | (buffer[3] << 8);
                Serial.print(F("[isConnected] FRAME OK! Dist:"));
                Serial.println(dist);
                delay(50);
                return true;
            }
        } else {
            // Print debug every 500ms if no data
            if (millis() - lastDebugTime > 500) {
                Serial.print(F("[isConnected] No data ("));
                Serial.print(millis() - startTime);
                Serial.println(F("ms)"));
                delay(50);
                lastDebugTime = millis();
            }
        }
        delay(50);  // Longer delay between attempts
    }
    
    // If we received bytes but no valid frame, might be baud rate or wiring issue
    Serial.print(F("[isConnected] TIMEOUT! Bytes:"));
    Serial.println(bytesReceived);
    delay(50);
    
    if (bytesReceived > 0) {
        Serial.println(F("[isConnected] Data but no frame!"));
        Serial.println(F("  Check: baud rate, TX/RX swap"));
        delay(50);
        
        // Try to dump first few bytes
        Serial.print(F("[isConnected] Hex: "));
        uint8_t dumpCount = min((uint32_t)10, (uint32_t)_serial->available());
        for (uint8_t i = 0; i < dumpCount && _serial->available(); i++) {
            uint8_t b = _serial->read();
            if (b < 0x10) Serial.print(F("0"));
            Serial.print(b, HEX);
            Serial.print(F(" "));
        }
        Serial.println();
        delay(50);
    } else {
        Serial.println(F("[isConnected] NO DATA!"));
        Serial.println(F("  Check: power, TX wire, GPIO pins"));
        delay(50);
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
    uint32_t bytesRead = 0;
    
    while (millis() - startTime < 100) {
        if (_serial->available()) {
            uint8_t byte = _serial->read();
            bytesRead++;
            
            switch (state) {
                case 0:  // Looking for first header byte (0x59)
                    if (byte == TF02_FRAME_HEADER) {
                        buffer[0] = byte;
                        state = 1;
                    }
                    break;
                    
                case 1:  // Looking for second header byte (0x59)
                    if (byte == TF02_FRAME_HEADER) {
                        buffer[1] = byte;
                        index = 2;
                        state = 2;
                    } else {
                        // Reset if second byte is not header
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






