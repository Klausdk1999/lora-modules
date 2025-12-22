/**
 * AJSR04M.h - AJ-SR04M Waterproof Ultrasonic Distance Sensor Driver
 * 
 * The AJ-SR04M is a waterproof ultrasonic sensor with:
 * - Range: 20cm to 800cm
 * - Accuracy: ±0.5cm
 * - Waterproof probe (IP67)
 * - Interface: Trigger/Echo GPIO pins
 * - Operating modes: GPIO mode (like HC-SR04) or Serial UART mode
 * 
 * This driver implements GPIO mode, compatible with HC-SR04 interface.
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef AJ_SR04M_H
#define AJ_SR04M_H

#include "../SensorBase.h"

// Speed of sound in cm/us at 20°C
#define SOUND_SPEED_CM_US   0.0343f

// Timeout for echo (in microseconds) - max ~50ms for 800cm
#define AJ_ECHO_TIMEOUT_US  60000

class AJSR04M : public SensorBase {
public:
    /**
     * Constructor
     * @param trigPin Trigger pin number
     * @param echoPin Echo pin number
     */
    AJSR04M(uint8_t trigPin, uint8_t echoPin) :
        _trigPin(trigPin), _echoPin(echoPin), _temperature(20.0f) {}
    
    /**
     * Constructor with temperature compensation
     * @param trigPin Trigger pin number
     * @param echoPin Echo pin number
     * @param tempCelsius Ambient temperature for speed-of-sound correction
     */
    AJSR04M(uint8_t trigPin, uint8_t echoPin, float tempCelsius) :
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
    
    SensorType getType() override { return SensorType::AJ_SR04M; }
    const char* getName() override { return "AJ-SR04M"; }
    float getMinDistance() override { return 20.0f; }   // 20 cm
    float getMaxDistance() override { return 800.0f; }  // 800 cm
    
    // ========================================================================
    // Additional AJ-SR04M specific methods
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
    float readDistanceAvg(uint8_t samples = 5, uint8_t delayMs = 100);
    
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

#endif // AJ_SR04M_H


