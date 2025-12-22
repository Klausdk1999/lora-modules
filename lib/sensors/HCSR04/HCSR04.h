/**
 * HCSR04.h - HC-SR04 Ultrasonic Distance Sensor Driver
 * 
 * The HC-SR04 is a common ultrasonic ranging sensor with:
 * - Range: 2cm to 400cm
 * - Accuracy: ±3mm
 * - Measuring angle: 15°
 * - Interface: Trigger/Echo GPIO pins
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef HC_SR04_H
#define HC_SR04_H

#include "../SensorBase.h"

// Speed of sound in cm/us at 20°C
#define SOUND_SPEED_CM_US   0.0343f

// Timeout for echo (in microseconds) - max ~25ms for 400cm
#define ECHO_TIMEOUT_US     30000

class HCSR04 : public SensorBase {
public:
    /**
     * Constructor
     * @param trigPin Trigger pin number
     * @param echoPin Echo pin number
     */
    HCSR04(uint8_t trigPin, uint8_t echoPin) :
        _trigPin(trigPin), _echoPin(echoPin), _temperature(20.0f) {}
    
    /**
     * Constructor with temperature compensation
     * @param trigPin Trigger pin number
     * @param echoPin Echo pin number
     * @param tempCelsius Ambient temperature for speed-of-sound correction
     */
    HCSR04(uint8_t trigPin, uint8_t echoPin, float tempCelsius) :
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
    
    SensorType getType() override { return SensorType::HC_SR04; }
    const char* getName() override { return "HC-SR04"; }
    float getMinDistance() override { return 2.0f; }    // 2 cm
    float getMaxDistance() override { return 400.0f; }  // 400 cm
    
    // ========================================================================
    // Additional HC-SR04 specific methods
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
    float readDistanceAvg(uint8_t samples = 5, uint8_t delayMs = 50);
    
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
    
    /**
     * Calculate speed of sound based on temperature
     * @return Speed of sound in cm/us
     */
    float getSpeedOfSound();
    
    /**
     * Send trigger pulse and measure echo
     * @return Echo duration in microseconds, or 0 if timeout
     */
    unsigned long measureEcho();
};

#endif // HC_SR04_H


