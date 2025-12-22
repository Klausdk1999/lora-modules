/**
 * AJSR04M.cpp - AJ-SR04M Waterproof Ultrasonic Sensor Implementation
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include "AJSR04M.h"

bool AJSR04M::begin() {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    
    // Set trigger low initially
    digitalWrite(_trigPin, LOW);
    delay(100);  // AJ-SR04M needs a bit more time to stabilize
    
    _initialized = true;
    return true;
}

bool AJSR04M::isConnected() {
    // Try to get a reading
    float dist = readDistanceCm();
    return (dist > 0 && dist <= getMaxDistance());
}

SensorReading AJSR04M::read() {
    SensorReading reading;
    reading.sensor_type = SensorType::AJ_SR04M;
    reading.timestamp = millis();
    
    if (!_initialized) {
        reading.valid = false;
        return reading;
    }
    
    float distance = readDistanceCm();
    
    if (distance < 0) {
        reading.valid = false;
        return reading;
    }
    
    // Validate range
    if (distance < getMinDistance() || distance > getMaxDistance()) {
        reading.valid = false;
        reading.distance_cm = distance;  // Store anyway for debugging
        return reading;
    }
    
    reading.valid = true;
    reading.distance_cm = distance;
    reading.distance_m = distance / 100.0f;
    reading.signal_strength = 0;  // Not available for ultrasonic
    reading.temperature = (int16_t)_temperature;
    
    return reading;
}

float AJSR04M::readDistanceCm() {
    unsigned long duration = measureEcho();
    
    if (duration == 0) {
        return -1;
    }
    
    _lastDuration = duration;
    
    // Calculate distance: distance = (time * speed) / 2
    float speedOfSound = getSpeedOfSound();
    float distance = (duration * speedOfSound) / 2.0f;
    
    return distance;
}

float AJSR04M::readDistanceAvg(uint8_t samples, uint8_t delayMs) {
    float sum = 0;
    uint8_t validSamples = 0;
    
    for (uint8_t i = 0; i < samples; i++) {
        float dist = readDistanceCm();
        
        if (dist > 0) {
            sum += dist;
            validSamples++;
        }
        
        if (i < samples - 1) {
            delay(delayMs);
        }
    }
    
    if (validSamples == 0) {
        return -1;
    }
    
    return sum / validSamples;
}

float AJSR04M::getSpeedOfSound() {
    // Speed of sound varies with temperature
    // Formula: c = 331.3 + 0.606 * T (where T is in Celsius)
    // Returns speed in m/s, convert to cm/us
    float speedMs = 331.3f + (0.606f * _temperature);
    return speedMs / 10000.0f;  // Convert m/s to cm/us
}

unsigned long AJSR04M::measureEcho() {
    // Ensure trigger is low
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(5);
    
    // Send 10us trigger pulse
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
    
    // Measure echo pulse duration
    // AJ-SR04M may need longer timeout due to 8m range
    unsigned long duration = pulseIn(_echoPin, HIGH, AJ_ECHO_TIMEOUT_US);
    
    return duration;
}


