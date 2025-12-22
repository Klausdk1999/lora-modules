/**
 * JSNSR04T.h - JSN-SR04T Waterproof Ultrasonic Distance Sensor Driver
 * 
 * The JSN-SR04T is a waterproof ultrasonic sensor with:
 * - Range: 25cm to 450cm
 * - Blind zone: <25cm
 * - Accuracy: ±1cm
 * - Waterproof probe (IP67)
 * - Interface: Trigger/Echo GPIO pins (Mode 1) or UART (Mode 2)
 * 
 * This driver implements Mode 1 (GPIO mode), compatible with HC-SR04 interface.
 * Mode 2 (UART) is automatically entered when R27 resistor is present.
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef JSN_SR04T_H
#define JSN_SR04T_H

#include "../SensorBase.h"

// Timeout for echo (in microseconds) - max ~28ms for 450cm
#define JSN_ECHO_TIMEOUT_US  35000

class JSNSR04T : public SensorBase {
public:
    /**
     * Constructor
     * @param trigPin Trigger pin number
     * @param echoPin Echo pin number
     */
    JSNSR04T(uint8_t trigPin, uint8_t echoPin) :
        _trigPin(trigPin), _echoPin(echoPin), _temperature(20.0f) {}
    
    /**
     * Constructor with temperature compensation
     * @param trigPin Trigger pin number
     * @param echoPin Echo pin number
     * @param tempCelsius Ambient temperature for speed-of-sound correction
     */
    JSNSR04T(uint8_t trigPin, uint8_t echoPin, float tempCelsius) :
        _trigPin(trigPin), _echoPin(echoPin), _temperature(tempCelsius) {}
    
    /**
     * Initialize the sensor pins
     * @return true always (no hardware detection possible)
     */
    bool begin() override;
    
    /**
     * Read distance measurement
     * @return SensorReading with measurement data
     */
    SensorReading read() override;
    
    /**
     * Check if sensor appears connected (reads valid distance)
     * @return true if valid reading obtained
     */
    bool isConnected() override;
    
    SensorType getType() override { return SensorType::JSN_SR04T; }
    const char* getName() override { return "JSN-SR04T"; }
    float getMinDistance() override { return 25.0f; }   // 25 cm (blind zone)
    float getMaxDistance() override { return 450.0f; }  // 450 cm
    
    // ========================================================================
    // Additional JSN-SR04T specific methods
    // ========================================================================
    
    /**
     * Read only distance (simple version)
     * @return Distance in centimeters, or -1 if timeout/error
     */
    float readDistanceCm();
    
    /**
     * Read distance with averaging
     * @param samples Number of samples to average
     * @param delayMs Delay between samples in milliseconds
     * @return Averaged distance in centimeters, or -1 if error
     */
    float readDistanceAvg(uint8_t samples = 5, uint8_t delayMs = 60);
    
    /**
     * Set ambient temperature for speed-of-sound correction
     * @param tempCelsius Temperature in Celsius
     */
    void setTemperature(float tempCelsius) { _temperature = tempCelsius; }
    
    /**
     * Get the current temperature setting
     * @return Temperature in Celsius
     */
    float getTemperature() { return _temperature; }
    
    /**
     * Get echo duration from last reading
     * @return Duration in microseconds
     */
    unsigned long getLastDuration() { return _lastDuration; }
    
private:
    uint8_t _trigPin;
    uint8_t _echoPin;
    float _temperature;
    unsigned long _lastDuration;
    
    float getSpeedOfSound();
    unsigned long measureEcho();
};

#endif // JSN_SR04T_H


